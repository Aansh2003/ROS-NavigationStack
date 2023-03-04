
# Pulling Docker Image 
docker pull ghcr.io/teak-rosewood/mrm-erc2022-navstack:latest

# Running Docker Image 
docker run -it --net=host -e ROS_IP -e ROS_MASTER_URI --env-file credentials.env --name mrm_img ghcr.io/teak-rosewood/mrm-erc2022-navstack

