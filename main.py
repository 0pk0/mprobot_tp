#!/usr/bin/env python3
"""
Main Script for Trajectory Tracking System

Command-line interface for running different trajectory tracking scenarios.
"""

import argparse
import sys
import os


def main():
    parser = argparse.ArgumentParser(
        description='Trajectory Tracking System - Path Smoothing and Pure Pursuit Control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py --scenario straight_line
  python main.py --scenario curved_path
  python main.py --scenario sharp_turn
  python main.py --scenario obstacle_avoidance
  python main.py --list-scenarios
        """
    )
    
    parser.add_argument(
        '--scenario',
        type=str,
        choices=['straight_line', 'curved_path', 'sharp_turn', 'obstacle_avoidance'],
        help='Scenario to run'
    )
    
    parser.add_argument(
        '--list-scenarios',
        action='store_true',
        help='List all available scenarios'
    )
    
    args = parser.parse_args()
    
    if args.list_scenarios:
        print("\nAvailable Scenarios:")
        print("=" * 60)
        print("1. straight_line       - Simple straight line path")
        print("2. curved_path         - Smooth curved path")
        print("3. sharp_turn          - Path with 90-degree turn")
        print("4. obstacle_avoidance  - Path with obstacles (BONUS FEATURE)")
        print("=" * 60)
        return
    
    if not args.scenario:
        parser.print_help()
        print("\nPlease specify a scenario with --scenario <name>")
        print("Use --list-scenarios to see available scenarios")
        return
    
    # Import and run the selected scenario
    print(f"\nRunning scenario: {args.scenario}")
    print("=" * 60)
    
    scenario_path = os.path.join('scenarios', f'{args.scenario}.py')
    
    if not os.path.exists(scenario_path):
        print(f"Error: Scenario file not found: {scenario_path}")
        return
    
    # Execute the scenario module
    import importlib.util
    spec = importlib.util.spec_from_file_location(args.scenario, scenario_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    # Run the scenario
    if hasattr(module, 'main'):
        module.main()
    else:
        print(f"Error: Scenario {args.scenario} does not have a main() function")


if __name__ == "__main__":
    main()
