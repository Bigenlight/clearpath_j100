# setup.py — j100_teleop 패키지 빌드 정의
# =========================================
# ament_python 빌드 타입 패키지의 핵심 빌드 스크립트.
# 'colcon build' 실행 시 이 파일을 읽어 패키지를 install space에 설치한다.
#
# [주요 역할]
#   1. 패키지 Python 모듈을 site-packages에 설치
#   2. package.xml / 리소스 마커를 share/ 디렉터리에 복사
#   3. entry_points로 'ros2 run' 에서 쓸 실행 가능한 스크립트를 등록

# setuptools.setup: 패키지 설치 메타데이터와 파일 목록을 colcon/pip에 전달하는 함수.
# find_packages: 패키지 디렉터리(__init__.py 포함)를 자동 탐색하는 헬퍼.
from setuptools import find_packages, setup

# ── 패키지 이름 상수 ────────────────────────────────────────────────────────────
# 여러 곳에서 반복 사용되는 패키지 이름을 한 곳에서 관리.
# package.xml의 <name> 태그와 반드시 일치해야 한다.
package_name = 'j100_teleop'

setup(
    # ── 기본 메타데이터 ─────────────────────────────────────────────────────────
    name=package_name,
    version='0.1.0',
    maintainer='Theo',
    maintainer_email='tpingouin@gmail.com',
    description='J100 (Jackal) 로봇용 PyQt5 가상 조이스틱 텔레옵 UI 패키지',
    license='Apache-2.0',

    # ── Python 패키지 목록 ──────────────────────────────────────────────────────
    # find_packages(): __init__.py가 있는 디렉터리를 자동 탐색.
    #   exclude=['test']: test/ 디렉터리는 배포에 포함하지 않음.
    # 결과적으로 ['j100_teleop'] 리스트가 된다.
    # (j100_teleop/ 디렉터리 내의 __init__.py + joystick_ui.py 등이 포함됨)
    packages=find_packages(exclude=['test']),

    # ── 데이터 파일 (non-Python 파일) 설치 경로 ────────────────────────────────
    # 형식: (설치_대상_디렉터리, [원본_파일_목록])
    # colcon은 이 목록을 보고 install/share/ 아래에 파일을 복사한다.
    data_files=[

        # [ament 리소스 인덱스 마커]
        # install/share/ament_index/resource_index/packages/j100_teleop (빈 파일)
        # ament_index_python.get_resources('packages') 등이 이 마커를 읽어
        # 패키지가 설치되어 있는지 확인한다.
        # 이 항목이 없으면 'ros2 pkg list' 에 패키지가 나타나지 않는다.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # [package.xml 설치]
        # install/share/j100_teleop/package.xml 위치에 복사.
        # 런타임에 rclpy 등이 패키지 메타데이터(의존성, 버전 등)를 읽을 때 사용.
        ('share/' + package_name, ['package.xml']),
    ],

    # ── Python 패키지 의존성 ────────────────────────────────────────────────────
    # pip install 수준의 의존성. ROS2 환경에서는 package.xml의 exec_depend가
    # 주된 의존성 관리 수단이므로 여기서는 setuptools만 명시하는 것이 관례.
    install_requires=['setuptools'],

    # ── zip_safe ────────────────────────────────────────────────────────────────
    # False: 패키지를 .egg 아카이브가 아니라 디렉터리 형태로 설치.
    # ROS2 패키지는 런타임에 파일 경로를 직접 참조하는 경우가 많으므로
    # zip으로 묶으면 문제가 생길 수 있어 False로 설정한다.
    zip_safe=False,

    # ── 테스트 의존성 ───────────────────────────────────────────────────────────
    tests_require=['pytest'],

    # ── 콘솔 스크립트 진입점 (entry_points) ────────────────────────────────────
    # 'console_scripts' 항목에 등록된 각 줄은
    # install/lib/j100_teleop/ 아래에 실행 가능한 스크립트로 설치된다.
    # (정확한 경로는 setup.cfg의 script_dir / install_scripts 설정에 따름)
    #
    # 형식: '실행명 = 모듈.경로:함수명'
    #
    # 'joystick_ui = j100_teleop.joystick_ui:main' 의 의미:
    #   ┌─ 좌변 'joystick_ui'
    #   │   → 'ros2 run j100_teleop joystick_ui' 명령에서 사용될 실행 파일 이름.
    #   │     colcon이 이 이름으로 래퍼 스크립트를 lib/j100_teleop/joystick_ui에 생성.
    #   │
    #   └─ 우변 'j100_teleop.joystick_ui:main'
    #       → 모듈 경로 'j100_teleop.joystick_ui' (j100_teleop/joystick_ui.py)
    #         + ':' (구분자)
    #         + 'main' (해당 모듈 안에서 호출할 함수 이름)
    #         즉, joystick_ui.py 안의 main() 함수가 진입점이 된다.
    entry_points={
        'console_scripts': [
            'joystick_ui = j100_teleop.joystick_ui:main',
        ],
    },
)
