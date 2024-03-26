# Crescendo Season Prototypes

This repo is meant to store all of our software prototypes for the crescendo season if your looking for the finished versions they are located in our Crescendo repository.

The command below will retriveve all the dependencies for each project.  

```sh
.\gradlew build
```

## Climber Prototype

This was the code written for the climbers for `robot_1`. It includes the set climber command & the climber subsystem.

### Deploying

Powershell

``` sh
.\gradlew projects:2k24_climber:2k24_Climber:deploy
```

## Shooter Prototype

This shooter prototype uses shuffleboard to retrieve the desired rpms for both top & bottom flywheels.

### Deploying

Powershell

``` sh
.\gradlew projects:2k24_climber:2k24_Climber:deploy
```

## Turret Prototype

Our earily turret prototype using sparkmax pid controller.

### Deploying

Powershell

``` sh
.\gradlew projects:2k24-Turret-Prototype:2k24_Turret:deploy
```

## IR Switch

Simple IR switch for the climbers on robot 2.

### Deploying

Powershell

``` sh
.\gradlew projects:IRSwitch:deploy
```

## Limelight Prototype

Code from our vision cart from week three. We were messing around with pose estimation using the limelight. Unfortunatly we didn't have enough time for full pose estimation.

### Deploying

Powershell

``` sh
.\gradlew projects:Limelight-Prototype:deploy
```

## Northstar Prototype

Code from our attempt with northstar vision system created by 6328.

### Deploying

Powershell

``` sh
.\gradlew projects:Northstar_robot:deploy
```
