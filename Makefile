install_deps:
	./docker/scripts/install_deps.sh 

test_docker:
	docker build --target vision_project_test --build-arg vision_project_branch=master --tag test docker/
	docker run test us-west-1 json ${AWS_SECRET_KEY_ID} ${AWS_SECRET_KEY}
