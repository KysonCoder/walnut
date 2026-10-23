#!/bin/bash
# 队列水位单行监视（每 0.5s）
watch -n 0.5 'curl -s http://127.0.0.1:8080/queueStatus | python3 -c "import json,sys;d=json.load(sys.stdin);print(d[\"runningMode\"],d[\"currentState\"],\"manual=%d program=%d traj=%d micro=%d\"%(d[\"manualList\"][\"size\"],d[\"programList\"][\"size\"],d[\"trajList\"][\"size\"],d[\"microList\"][\"size\"]),\"fb[:7]=\",[round(x,3) for x in d[\"feedback\"][:7]])"'
