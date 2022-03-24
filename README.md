소스 빌드 사용 매뉴얼
=================

workspaces : 전체 Workspace

- ros2_galactic : ROS2 galactic 기본 workspace
- self_drive_ws : 플랫폼 구동을 위한 workspace
- simulation_ws : gazebo 시뮬레이션 환경 구동을 위한 workspace

bin_packages : 테스트용 패키지, Mbed 펌웨어, 기본 패키지 템플릿 등이 포함된 폴더

<br/><br/>

amd64 / ubuntu 18.04, 20.04   기준 workspaces 빌드 테스트 진행 완료.

aarch64 / Jetpack 4.6에서 빌드 실패, Jetpack 4.5.1에서 빌드 테스트 진행 완료.

>   ** amd64 : 현존하는 대부분의 64비트 기반 CPU 아키텍쳐. Intel, AMD 계열 CPU를 포함함.

>   ** aarch64 : ARM 기반 CPU 아키텍쳐. 모든 NVIDIA Jetson 계열에서 채택하는 CPU 아키텍쳐임.

<br/><br/>

권장 소프트웨어 : Github Desktop (https://github.com/shiftkey/desktop/releases)

clone repository - URL - URL 입력 바에 https://github.com/parkdg44/Platform_ROS2_ws.git 입력 - clone

<br/>
