#!/usr/bin/env python3
import time, unittest
from swarm_uav_interfaces.srv import UavSafetyLeaseResponse
from tcp_to_ros.safety_lease import SafetyLeaseCoordinator

class CoordinatorTest(unittest.TestCase):
    def test_concurrent_start_renew_end(self):
        calls=[]
        def call(target,req,_timeout): calls.append((target,req.operation,req.lease_seq)); return UavSafetyLeaseResponse(True,"ACTIVE","","")
        item=SafetyLeaseCoordinator(call,epoch_factory=lambda:"epoch")
        self.assertEqual([],item.begin_session("m",[("A01","UAV1"),("A02","UAV2")],time.monotonic()+1))
        self.assertEqual([],item.renew()); self.assertEqual([],item.end_session(time.monotonic()+1))
        self.assertIsNone(item.session); self.assertIn(("UAV1","RENEW",2),calls)
    def test_partial_start_is_compensated(self):
        calls=[]
        def call(target,req,_timeout):
            calls.append((target,req.operation,req.lease_seq))
            return UavSafetyLeaseResponse(target!="UAV2","ACTIVE","REJECTED","no")
        item=SafetyLeaseCoordinator(call,epoch_factory=lambda:"epoch")
        self.assertTrue(item.begin_session("m",[("A01","UAV1"),("A02","UAV2")],time.monotonic()+1))
        self.assertIn(("UAV1","END",2),calls); self.assertIsNone(item.session)
    def test_calls_are_parallel(self):
        def call(_target,_req,_timeout): time.sleep(.08); return UavSafetyLeaseResponse(True,"ACTIVE","","")
        item=SafetyLeaseCoordinator(call)
        start=time.monotonic(); item.begin_session("m",[(str(i),"UAV"+str(i)) for i in range(10)],time.monotonic()+1)
        self.assertLess(time.monotonic()-start,.25)
    def test_late_start_response_cannot_revive_session(self):
        def call(_target,_req,_timeout): time.sleep(.05); return UavSafetyLeaseResponse(True,"ACTIVE","","")
        item=SafetyLeaseCoordinator(call,call_timeout_s=.01)
        failures=item.begin_session("m",[("A01","UAV1")],time.monotonic()+.01)
        self.assertTrue(failures); self.assertEqual("LEASE_CALL_TIMEOUT",failures[0].error_code)
        time.sleep(.06); self.assertIsNone(item.session)
    def test_failed_renew_is_retried(self):
        renew=[]
        def call(target,req,_timeout):
            if req.operation=="RENEW":
                renew.append(target)
                if len(renew) == 1:
                    return UavSafetyLeaseResponse(False,"ACTIVE","NO_ACK","no")
            return UavSafetyLeaseResponse(True,"ACTIVE","","")
        item=SafetyLeaseCoordinator(call); item.begin_session("m",[("A01","UAV1")],time.monotonic()+1)
        self.assertTrue(item.renew()); self.assertEqual([],item.renew()); self.assertEqual(["UAV1", "UAV1"],renew)
    def test_reuses_epoch_and_starts_only_new_members(self):
        calls=[]
        def call(target,req,_timeout): calls.append((target,req.operation,req.session_epoch)); return UavSafetyLeaseResponse(True,"ACTIVE","","")
        item=SafetyLeaseCoordinator(call,epoch_factory=lambda:"epoch")
        self.assertEqual([],item.begin_session("m",[("A01","UAV1")],time.monotonic()+1))
        self.assertEqual([],item.begin_session("m",[("A01","UAV1"),("A02","UAV2")],time.monotonic()+1))
        self.assertEqual([("UAV1","START","epoch"),("UAV2","START","epoch")],calls)
if __name__=="__main__": unittest.main()