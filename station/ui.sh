clear

~/drone/trunk/ARDrone_sdk/Examples/Linux/Build/Release/linux_sdk_demo &
python ~/drone/trunk/station/planner.py &
python ~/drone/trunk/station/ui.py &
python ~/drone/trunk/station/pso.py

vidpid=`ps h -o pid -C $STRING linux_sdk_demo`
echo $vidpid
kill $vidpid
 

