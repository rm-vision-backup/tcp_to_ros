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
            if self._session is None:
                session=LeaseSession(mission_id,self.epoch_factory(),dict(mapping),{u:1 for u in mapping})
                self._session=session; new_targets=dict(mapping); new_session=True
            else:
                session=self._session; new_session=False
                if session.mission_id != mission_id:
                    return [LeaseFailure("","","START","SESSION_ACTIVE","another mission lease is active")]
                conflicts=[u for u,t in mapping.items() if u in session.participants and session.participants[u]!=t]
                if conflicts:
                    return [LeaseFailure(u,mapping[u],"START","LEASE_IDENTITY_MISMATCH","participant identity changed") for u in conflicts]
                new_targets={u:t for u,t in mapping.items() if u not in session.participants}
                for u,t in new_targets.items(): session.participants[u]=t; session.sequences[u]=1
                if not new_targets: return []
        failures=self._parallel(session,"START",new_targets,deadline)
        if failures:
            with self._lock:
                started=set(new_targets).intersection(session.active)
                for uav_id in started:
                    session.sequences[uav_id] += 1
            compensation_deadline = self.clock() + self.call_timeout_s
            self._parallel(session,"END",{u:new_targets[u] for u in started},compensation_deadline)
            with self._lock:
                for uav_id in new_targets:
                    session.participants.pop(uav_id,None); session.sequences.pop(uav_id,None)
                if new_session: self._session=None
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
        failures=[]; failure_keys=set(); lock=threading.Lock(); threads=[]
        def add_failure(failure):
            key=(failure.uav_id,failure.operation)
            with lock:
                if key not in failure_keys:
                    failure_keys.add(key); failures.append(failure)
        def worker(uav_id, target):
            remaining=max(0.0,deadline-self.clock())
            request=UavSafetyLeaseRequest("1.0",operation,session.mission_id,session.epoch,
                session.sequences[uav_id],self.ttl_s if operation!="END" else 0.0,target,target,operation.lower())  # uav_id=exec_target UAVn
            try:
                response=self.call(target,request,min(self.call_timeout_s,remaining))
                if not response.accepted: raise RuntimeError("%s: %s"%(response.error_code,response.message))
                with self._lock:
                    if self.clock() > deadline:
                        add_failure(LeaseFailure(uav_id,target,operation,"LEASE_CALL_TIMEOUT","late response discarded"))
                        return
                    if self._session is not session:
                        return
                    if operation=="START": session.active.add(uav_id)
                    elif operation=="END": session.active.discard(uav_id)
            except Exception as error:
                code=getattr(error,"error_code","LEASE_%s_FAILED"%operation)
                if operation == "RENEW":
                    with self._lock:
                        session.active.discard(uav_id)
                add_failure(LeaseFailure(uav_id,target,operation,code,str(error)))
        for item in targets.items():
            thread=threading.Thread(target=worker,args=item,daemon=True); thread.start(); threads.append(thread)
        for thread in threads: thread.join(max(0.0,deadline-self.clock()))
        for thread,(uav_id,target) in zip(threads,targets.items()):
            if thread.is_alive():
                if operation == "RENEW":
                    with self._lock:
                        session.active.discard(uav_id)
                add_failure(LeaseFailure(uav_id,target,operation,"LEASE_CALL_TIMEOUT","deadline exceeded"))
        return failures