import yaml
import os
from ament_index_python.packages import get_package_share_directory

def main():
    x=5
    swarm_dir = get_package_share_directory('swarm_navigation')
    with open(os.path.join(swarm_dir, 'params',f'nav2_multirobot_params_{x}.yaml'), 'w+') as yaml_file:
            print(yaml_file)
            yaml.dump(x,yaml_file)
            yaml_file.close()


if __name__ == "__main__":
    main()