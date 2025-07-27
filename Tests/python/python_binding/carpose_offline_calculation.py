import os
import sys
import argparse
from pathlib import Path

# Add the path to import Classes module
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import Classes

def main():
    """
    Main function for loading AI-driver trip data.
    This script implements part of B7 from the localization separation plan:
    B7: Create Python test script for loading AI-driver trip data
    """
    # Setup argument parser with only necessary arguments
    parser = argparse.ArgumentParser(description='Load AI-driver trip data using Classes.Trip.')
    parser.add_argument('--trip_path', type=str, 
                        default='/home/eranvertz/imagry/trips/RTK_Haifa/2024-10-28T16_02_54​/',
                        help='Path to the trip data directory')
    args = parser.parse_args()
    
    # Initialize the Trip object to load data from experiment
    aidriver_trip_path = Path(args.trip_path)
    print('Loading trip data from ' + str(aidriver_trip_path))
    trip_obj = Classes.Trip(aidriver_trip_path)
    print('Trip data loaded successfully')
    
    # Print some basic trip information
    print(f"Trip duration: {trip_obj.common_time[-1]:.2f} seconds")
    print(f"Number of samples: {len(trip_obj.common_time)}")
    print(f"Sample rate: {len(trip_obj.common_time) / trip_obj.common_time[-1]:.2f} Hz")
    
    print("Script completed. Trip data loaded.")
    
    trip_obj.sort_localization_inputs()
    
    
if __name__ == '__main__':
    main()
