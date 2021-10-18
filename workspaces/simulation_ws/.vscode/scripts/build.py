import argparse
import os

parser = argparse.ArgumentParser(description="path")
parser.add_argument('path', metavar='path', type=str, nargs='+',
                    help='path to execute file.')

if __name__ == "__main__":
    args = parser.parse_args()
    str_path = str(args.path)    
    str_s = str_path.split('/')
    package_name = str_s[-3]    
    os.system("colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select " + package_name)
    
    
