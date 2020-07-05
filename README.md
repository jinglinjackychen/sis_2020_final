# Take Hello world as example

## About this example

You can see commit "hello world".

## Path:

> competition_modules

>> core
    
>> place_to_box
     
>> pose_estimate_and_pick   
    
>> robot_navigation
    
>> object_detection
              
> README.md

> Dockerfile            (You don't need to modify this file)

> run_task.sh           (You don't need to modify this file)

> docker_run.sh         (You don't need to modify this file)

> catkin_make.sh        (You don't need to modify this file)

> environment.sh        (You don't need to modify this file)

> task.launch    (You have to determine which node you need to launch in the task and write in this file)

> docker_build.sh       (If you want to build docker file, please execute/source this shell)

## How to run hello world:

***Step1:build image***

tx2 $ cd [your sis_competition_template path]

tx2 $ source docker_build.sh

***If docker is already login with other account, please logout first.***

tx2 $ docker logout

***Type your dockerhub's account and password.***

tx2 $ docker login

tx2 $ docker tag sis_competition_2020:latest sis_competition_2020:hello_world

***Step2:run task***

tx2 $ cd [your sis_competition_template path]

tx2 $ source docker_run.sh hello_world

***After enter container, you need to run this command once.***

tx2 docker $ source catkin_make.sh 

***Run task***

tx2 docker $ source run_task.sh

***If you want to enter same container, run this.***

tx2 $ source docker_run.sh same

tx2 docker $ source environment.sh (remember!！)
# sis_2020_final
