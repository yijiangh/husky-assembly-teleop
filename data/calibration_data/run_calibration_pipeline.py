"""
Meta script to run the complete calibration pipeline.

This script runs all four calibration scripts in sequence:
1. 0_circle_fitting.py - Fit circles to mocap data for j0 and j1 joints
2. 1_calibration_analysis.py - Analyze circle fits and compute base frame
3. 2_visualize_transformation.py - Visualize and save calibrated transformation
4. 3_verify_calibration.py - Verify calibration quality

Usage:
    python run_calibration_pipeline.py
"""

import os
import sys
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# Scripts to run in order
SCRIPTS = [
    ('0_circle_fitting.py', 'Circle Fitting'),
    ('1_calibration_analysis.py', 'Calibration Analysis'),
    ('2_convert_and_visualize_transformation.py', 'Convert and Visualize Transformation'),
    ('3_verify_calibration.py', 'Verify Calibration'),
]


def run_script(script_name, python_exe=None):
    """Run a Python script and return success status."""
    if python_exe is None:
        python_exe = sys.executable
    
    script_path = os.path.join(HERE, script_name)
    
    if not os.path.exists(script_path):
        print(f"  ERROR: Script not found: {script_path}")
        return False
    
    try:
        result = subprocess.run(
            [python_exe, script_path],
            cwd=HERE,
            capture_output=False,  # Show output in real-time
            text=True
        )
        return result.returncode == 0
    except Exception as e:
        print(f"  ERROR: Failed to run {script_name}: {e}")
        return False


def main():
    print("=" * 70)
    print("CALIBRATION PIPELINE")
    print("=" * 70)
    
    import argparse

    parser = argparse.ArgumentParser(
        description="Run the calibration pipeline. Optionally specify a date folder."
    )
    parser.add_argument(
        "date_folder",
        nargs="?",
        default=None,
        help="Name of the date folder to use (optional; defaults to DEFAULT_DATE_FOLDER in config_loader.py)"
    )
    args = parser.parse_args()

    date_folder = args.date_folder
    if date_folder is not None:
        print(f"Using date folder: {date_folder}")
        # Update DEFAULT_DATE_FOLDER would require modifying config_loader
        # For now, just inform the user
        print("Note: Make sure config_loader.py DEFAULT_DATE_FOLDER matches this.")

    print()
    total_start = time.time()
    results = []
    
    for i, (script_name, description) in enumerate(SCRIPTS, 1):
        print(f"\n{'=' * 70}")
        print(f"Step {i}/4: {description}")
        print(f"Running: {script_name}")
        print("=" * 70)
        
        start_time = time.time()
        success = run_script(script_name)
        elapsed = time.time() - start_time
        
        status = "SUCCESS" if success else "FAILED"
        results.append((script_name, success, elapsed))
        
        print(f"\n[{status}] {script_name} completed in {elapsed:.2f}s")
        
        if not success:
            print(f"\nPipeline stopped due to failure in {script_name}")
            break
    
    # Summary
    total_elapsed = time.time() - total_start
    
    print("\n" + "=" * 70)
    print("PIPELINE SUMMARY")
    print("=" * 70)
    
    for script_name, success, elapsed in results:
        status = "✓" if success else "✗"
        print(f"  {status} {script_name}: {elapsed:.2f}s")
    
    print(f"\nTotal time: {total_elapsed:.2f}s")
    
    all_success = all(r[1] for r in results)
    if all_success and len(results) == len(SCRIPTS):
        print("\n✓ All steps completed successfully!")
        return 0
    else:
        print("\n✗ Pipeline did not complete successfully.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
