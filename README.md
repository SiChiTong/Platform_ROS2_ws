소스 빌드 사용 매뉴얼
=================

workspaces : ros2 foxy 버전의 ws

workspaces_dashing : ros2 dashing 전용 ws.     

>   ** ros2 foxy : ubuntu 20.04(focal fossa)에 대응되는 ros2 버전.

>   ** ros2 dashing : ubuntu 18.04(bionic)에 대응되는 ros2 버전. 

>   ** ws를 구분한 이유는 jetpack이 현재 ubuntu 18.04까지만 지원하기 때문에 혹시 모를 버전 문제를 대비하기 위해서임.

>   ** jetpack : NVIDIA Jetson 계열 임베디드 보드를 위한 통합 라이브러리 패키지.  

>   ** 기본적으로 소스 빌드는 ubuntu 버전에 관계없이 동작함.

<br/><br/>

amd64 / ubuntu 18.04 LTS 기준 workspaces, workspaces_dashing 에 대해 빌드 테스트 진행 완료.

aarch64 / Jetpack 4.6에서 빌드 실패, Jetpack 4.5.1에서 빌드 테스트 진행 완료.

>   ** amd64 : 현존하는 대부분의 64비트 기반 CPU 아키텍쳐. Intel, AMD 계열 CPU를 포함함.

>   ** aarch64 : ARM 기반 CPU 아키텍쳐. 모든 NVIDIA Jetson 계열에서 채택하는 CPU 아키텍쳐임.

<br/><br/>

진행상황 (테스트배드 플랫폼 기준)

ROS2 통합 : 85%
>   다음 작업 : 피드백 제어 구현 / Nav2 기능 구현

자율 주행 : 80%
>   다음 작업 : Nav2 기능 구현

딥러닝 : 60%
>   다음 작업 : 신경망 추가 학습 및 장애물 위치 인식 방안 고안

플랫폼 제어 : 85%
>   다음 작업 : 피드백 제어 구현

GUI / APP : 40%
>   다음 작업 : 핵심 기능 구현(지도 및 Status 표시 등)

