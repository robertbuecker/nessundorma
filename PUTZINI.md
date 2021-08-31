#Putzini commands

Start server on Putzini:
`ssh putzini@192.168.1.19 'tmux new-session -d -s hover-py "cd nessundorma; ./hover.py"'`
Password is `lovearka`, but consider installing your public key.

Useful aruco commands, to be run in NX session:
`aruco_dcf_mm live:0 aruco_xpon.yml calib_usbgs/usbgs.yml -s 0.192`

###Movement
* `setReferencePos()`