$proj = -join("proj:"+,$args[0])
$ver = -join("ver:"+,$args[1])

echo $proj
echo $ver
$vernum = "newVer"
$project = "roban"


node C:\work\data\leju_src\firmwareci\main.js $vernum $project $args[0] $args[1]
python C:\work\data\leju_src\updateConfig\updateConfig.py $CI_PROJECT_NAME $CI_COMMIT_SHA