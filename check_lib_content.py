import Rosmaster_Lib.Rosmaster_Lib as R
import inspect

print(f"Has FUNC_REPORT_ICM_RAW: {hasattr(R.Rosmaster, 'FUNC_REPORT_ICM_RAW')}")
if hasattr(R.Rosmaster, 'FUNC_REPORT_ICM_RAW'):
    print(f"Value: {R.Rosmaster.FUNC_REPORT_ICM_RAW}")

src = inspect.getsource(R.Rosmaster.__init__)
print("Init source start:")
print(src[:500])
