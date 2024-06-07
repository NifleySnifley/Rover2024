rm -rf ./build
rm -rf ./install
docker build --platform=linux/aarch64 --tag=nifleysnifley/rover2024 . --push -f ./robot_docker/Dockerfile