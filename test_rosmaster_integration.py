#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosmaster 인터페이스 테스트 스크립트
하드웨어 연결 없이 import 및 기본 기능 확인
"""

import sys

def test_rosmaster_import():
    """Rosmaster_Lib import 테스트"""
    print("=" * 60)
    print("1. Rosmaster_Lib Import 테스트")
    print("=" * 60)
    
    try:
        from Rosmaster_Lib import Rosmaster
        print("✓ Rosmaster_Lib import 성공")
        
        # 사용 가능한 메서드 확인
        methods = [m for m in dir(Rosmaster) if not m.startswith('_')]
        print(f"✓ 사용 가능한 public 메서드: {len(methods)}개")
        
        # 주요 메서드 확인
        key_methods = [
            'create_receive_threading',
            'set_car_motion',
            'get_accelerometer_data',
            'get_gyroscope_data',
            'get_magnetometer_data',
            'get_motion_data',
            'get_battery_voltage',
            'set_beep',
            'set_auto_report_state',
            'set_pid_param'
        ]
        
        print("\n주요 메서드 확인:")
        for method in key_methods:
            if hasattr(Rosmaster, method):
                print(f"  ✓ {method}")
            else:
                print(f"  ✗ {method} - 없음!")
                return False
        
        return True
        
    except ImportError as e:
        print(f"✗ Rosmaster_Lib import 실패: {e}")
        return False
    except Exception as e:
        print(f"✗ 예상치 못한 오류: {e}")
        return False

def test_transbot_driver_import():
    """TransbotDriver import 테스트"""
    print("\n" + "=" * 60)
    print("2. TransbotDriver Import 테스트")
    print("=" * 60)
    
    try:
        # ROS2가 초기화되지 않은 상태에서 import만 테스트
        sys.path.insert(0, '/home/jetson/transbot_ws_ros2/src/transbot_bringup')
        from transbot_bringup.transbot_driver import TransbotDriver
        print("✓ TransbotDriver import 성공")
        
        # 클래스 속성 확인
        if hasattr(TransbotDriver, '__init__'):
            print("✓ TransbotDriver 클래스 정상")
        
        return True
        
    except ImportError as e:
        print(f"✗ TransbotDriver import 실패: {e}")
        print("  (참고: ROS2 패키지는 source install/setup.bash 후 정상 작동)")
        return False
    except Exception as e:
        print(f"✗ 예상치 못한 오류: {e}")
        return False

def test_api_compatibility():
    """API 호환성 확인"""
    print("\n" + "=" * 60)
    print("3. API 호환성 확인")
    print("=" * 60)
    
    try:
        from Rosmaster_Lib import Rosmaster
        
        # 가상 객체로 메서드 시그니처 확인
        print("\n메서드 시그니처 확인:")
        
        # set_car_motion 시그니처
        import inspect
        sig = inspect.signature(Rosmaster.set_car_motion)
        print(f"  set_car_motion{sig}")
        params = list(sig.parameters.keys())
        if 'v_x' in params and 'v_y' in params and 'v_z' in params:
            print("    ✓ 올바른 파라미터 (v_x, v_y, v_z)")
        
        # get_motion_data 시그니처
        sig = inspect.signature(Rosmaster.get_motion_data)
        print(f"  get_motion_data{sig}")
        
        # get_accelerometer_data 시그니처
        sig = inspect.signature(Rosmaster.get_accelerometer_data)
        print(f"  get_accelerometer_data{sig}")
        
        print("\n✓ API 호환성 확인 완료")
        return True
        
    except Exception as e:
        print(f"✗ API 확인 실패: {e}")
        return False

def check_serial_port():
    """시리얼 포트 확인"""
    print("\n" + "=" * 60)
    print("4. 시리얼 포트 확인")
    print("=" * 60)
    
    import os
    
    ports = ['/dev/ttyTHS1', '/dev/ttyTHS2', '/dev/ttyUSB0']
    
    for port in ports:
        if os.path.exists(port):
            # 권한 확인
            import stat
            st = os.stat(port)
            mode = st.st_mode
            
            readable = bool(mode & stat.S_IRGRP) or bool(mode & stat.S_IROTH)
            writable = bool(mode & stat.S_IWGRP) or bool(mode & stat.S_IWOTH)
            
            status = "✓" if (readable and writable) else "⚠"
            print(f"  {status} {port} - 존재함", end="")
            
            if not (readable and writable):
                print(" (권한 필요: sudo usermod -a -G dialout $USER)")
            else:
                print()
        else:
            print(f"  ✗ {port} - 없음")
    
    return True

def main():
    """메인 테스트 실행"""
    print("\n" + "=" * 60)
    print(" Rosmaster_Lib 인터페이스 전환 검증")
    print("=" * 60 + "\n")
    
    results = []
    
    # 1. Rosmaster import 테스트
    results.append(("Rosmaster Import", test_rosmaster_import()))
    
    # 2. TransbotDriver import 테스트
    results.append(("TransbotDriver Import", test_transbot_driver_import()))
    
    # 3. API 호환성 확인
    results.append(("API 호환성", test_api_compatibility()))
    
    # 4. 시리얼 포트 확인
    results.append(("시리얼 포트", check_serial_port()))
    
    # 결과 요약
    print("\n" + "=" * 60)
    print(" 테스트 결과 요약")
    print("=" * 60)
    
    for test_name, result in results:
        status = "✓ 통과" if result else "✗ 실패"
        print(f"  {status}: {test_name}")
    
    all_passed = all(result for _, result in results)
    
    print("\n" + "=" * 60)
    if all_passed:
        print(" ✓ 모든 테스트 통과!")
        print(" 다음 단계: 하드웨어 연결 후 실제 구동 테스트")
    else:
        print(" ⚠ 일부 테스트 실패")
        print(" 실패한 항목을 확인하고 수정하세요.")
    print("=" * 60 + "\n")
    
    return 0 if all_passed else 1

if __name__ == '__main__':
    sys.exit(main())
