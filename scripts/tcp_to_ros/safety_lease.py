"""Concurrent GCS coordinator for one UAV safety-lease session."""
from __future__ import annotations
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple
from swarm_uav_interfaces.srv import UavSafetyLeaseRequest

@dataclass(frozen=True)
class LeaseFailure:
    uav_id: str
    exec_target: str
    operation: str
    error_code: str
    message: str

@dataclass
class LeaseSession:
    mission_id: str
    epoch: str
    participants: Dict[str, str]
    sequences: Dict[str, int] = field(default_factory=dict)
    active: set = field(default_factory=set)

class SafetyLeaseCoordinator:
    def __init__(self, call, ttl_s=5.0, call_timeout_s=1.0,
                 epoch_factory=lambda: str(uuid.uuid4()), clock=time.monotonic):
        self.call=call; self.ttl_s=float(ttl_s); self.call_timeout_s=float(call_timeout_s)
        self.epoch_factory=epoch_factory; self.clock=clock; self._lock=threading.RLock(); self._session=None

    @property
    def session(self):
        with self._lock: return self._session

    def begin_session(self, mission_id: str, participants: Iterable[Tuple[str,str]], deadline: float):
        mapping=dict(participants)
        with self._lock:
            if self._session is not None: return [LeaseFailure("","","START","SESSION_ACTIVE","session already active")]
            session=LeaseSession(mission_id,self.epoch_factory(),mapping,{u:1 for u in mapping})
            self._session=session
        failures=self._parallel(session,"START",mapping,deadline)
        if failures:
            with self._lock:
                for uav_id in session.active:
                    session.sequences[uav_id] += 1
            self._parallel(session,"END",{u:mapping[u] for u in session.active},deadline)
            with self._lock: self._session=None
        return failures

    def renew(self, deadline: Optional[float]=None):
        with self._lock:
            session=self._session
            if session is None: return []
            targets={u:session.participants[u] for u in session.active}
            for u in targets: session.sequences[u]+=1
        return self._parallel(session,"RENEW",targets,deadline or self.clock()+self.call_timeout_s)

    def end_session(self, deadline: float):
        with self._lock:
            session=self._session
            if session is None: return []
            targets={u:session.participants[u] for u in session.active}
            for u in targets: session.sequences[u]+=1
        failures=self._parallel(session,"END",targets,deadline)
        if not failures:
            with self._lock: self._session=None
        return failures

    def _parallel(self, session, operation, targets, deadline):
        failures=[]; lock=threading.Lock(); threads=[]
        def worker(uav_id, target):
            remaining=max(0.0,deadline-self.clock())
            request=UavSafetyLeaseRequest("1.0",operation,session.mission_id,session.epoch,
                session.sequences[uav_id],self.ttl_s if operation!="END" else 0.0,uav_id,target,operation.lower())
            try:
                response=self.call(target,request,min(self.call_timeout_s,remaining))
                if not response.accepted: raise RuntimeError("%s: %s"%(response.error_code,response.message))
                with self._lock:
                    if self.clock() > deadline or self._session is not session:
                        return
                    if operation=="START": session.active.add(uav_id)
                    elif operation=="END": session.active.discard(uav_id)
            except Exception as error:
                code=getattr(error,"error_code","LEASE_%s_FAILED"%operation)
                if operation == "RENEW":
                    with self._lock:
                        session.active.discard(uav_id)
                with lock: failures.append(LeaseFailure(uav_id,target,operation,code,str(error)))
        for item in targets.items():
            thread=threading.Thread(target=worker,args=item,daemon=True); thread.start(); threads.append(thread)
        for thread in threads: thread.join(max(0.0,deadline-self.clock()))
        for thread,(uav_id,target) in zip(threads,targets.items()):
            if thread.is_alive():
                if operation == "RENEW":
                    with self._lock:
                        session.active.discard(uav_id)
                failures.append(LeaseFailure(uav_id,target,operation,"LEASE_CALL_TIMEOUT","deadline exceeded"))
        return failures