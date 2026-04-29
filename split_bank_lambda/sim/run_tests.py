import subprocess
import sys
import os

def run_tests():
    print("Building simulation...")
    # Compile both the test bench and the production logic
    build_cmd = ["g++", "split_bank_lambda/sim/test_sim.cpp", "split_bank_lambda/ControllerLogic.cpp", "-o", "split_bank_lambda/sim/test_bench"]
    try:
        subprocess.check_call(build_cmd)
    except subprocess.CalledProcessError:
        print("Build failed!")
        return False

    print("Running tests...")
    run_cmd = ["./split_bank_lambda/sim/test_bench"]
    result = subprocess.run(run_cmd, capture_output=True, text=True)

    print(result.stdout)

    # Cleanup binary
    if os.path.exists("split_bank_lambda/sim/test_bench"):
        os.remove("split_bank_lambda/sim/test_bench")

    if result.returncode == 0:
        print("PASSED")
        return True
    else:
        print("FAILED")
        print(result.stderr)
        return False

if __name__ == "__main__":
    if not run_tests():
        sys.exit(1)
