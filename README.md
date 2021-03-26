# AUV-Joystick
按照下面兩個網站安裝joy設置搖桿（http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick）並撰寫launch包(http://wiki.ros.org/joy/Tutorials/WritingTeleopNode)。
這個程式遇到的問題是，即時更新的遙感數據會因為要等待每一筆數據跑完，所以lag很嚴重。
解決方法是，把輸出的數據改為搖桿的上一筆資料，他就會即時更新移動狀態。
