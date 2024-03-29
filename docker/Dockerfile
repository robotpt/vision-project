# This Dockerfile builds multiple stages so it can be used for testing and development

# =======================================================
# The minimum requirements for running the vision-project
# =======================================================
FROM osrf/ros:kinetic-desktop-full as vision_project_min

# Pass in a command line arguement to set the branch to use
# e.g., docker build --build-arg vision_project_branch=master --target vision_project_dev .
ARG vision_project_branch="add-reading-task-data"
ARG vision_project_remote="https://github.com/robotpt/vision-project.git"
ARG INSTALL_SCRIPT="install_deps.sh"

# Install dependencies
COPY ./scripts/${INSTALL_SCRIPT} /tmp/${INSTALL_SCRIPT}
RUN /tmp/${INSTALL_SCRIPT}

# Setup the ROS workspace
ENV ROS_WS="/root/catkin_ws"
RUN \
	mkdir -p ${ROS_WS}/src \
	&& cd ${ROS_WS}/src \
	&& git clone --branch ${vision_project_branch} --recurse-submodules ${vision_project_remote} \
	&& git clone https://github.com/robotpt/cordial.git \
	&& git clone https://github.com/robotpt/qt-robot.git \
	&& git clone https://github.com/robotpt/ros-data-capture.git \
	&& git clone --branch ros1 https://github.com/RobotWebTools/rosbridge_suite.git \
	&& git clone https://github.com/audrow/mongodb-statedb.git \
	&& cd mongodb-statedb && /bin/bash -c "python3.8 -m pip install ." \
	&& cd ../..; /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"


# ==============================================================
# Copy the testing entrypoint for testing vision-project for PRs
# ==============================================================
FROM vision_project_min as vision_project_test_pr
COPY ./scripts/test_pr.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# ====================================================================
# Copy the testing entrypoint for testing vision-project for all tests
# ====================================================================
# Note that this runs AWS tests, so you must have your AWS credentials
# set
FROM vision_project_min as vision_project_test_full
COPY ./scripts/test_full.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]


# ===================================================
# A set of tools to help in developing vision-project
# ===================================================
FROM vision_project_min as vision_project_dev

# Set frontend to avoid debconf display errors
ENV DEBIAN_FRONTEND noninteractive

# Install essentials
RUN \
	apt-get update -y && apt-get upgrade -y \
	&& apt-get install -y \
		software-properties-common \
		apt-utils \
	&& apt-get update -y \
	&& apt-get install -y \
		curl \
		wget \
		tmux \
		zsh \
		vim \
		sudo \
		libgl1-mesa-glx \
		libgl1-mesa-dri \
		mesa-utils \
		unzip \
		terminator \
		git-gui \
		fonts-powerline \
		meld \
		nautilus \
		vlc \
	&& sed -i 's/geteuid/getppid/' /usr/bin/vlc \
	&& rm -rf /var/lib/apt/lists/* 

# Setup Pycharm
RUN \
	wget https://download.jetbrains.com/python/pycharm-community-2020.2.1.tar.gz?_ga=2.164454841.706163669.1598639929-1052883321.1598639929 \
	&& mv pycharm-community-* pycharm-community.tar.gz \
	&& tar xfz pycharm-*.tar.gz -C /opt/ \
	&& rm pycharm-community.tar.gz \
	&& echo 'alias pycharm="/opt/pycharm-community*/bin/pycharm.sh"' >> /root/.bashrc 

# Install Visual Studio Code
RUN \
	apt-get update -y \
	&& apt-get install -y \
		apt-transport-https \
		ca-certificates \
	&& curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg \
	&& mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg \
	&& sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list' \
	&& apt-get update -y \
	&& apt-get install -y code \
	&& rm -rf /var/lib/apt/lists/* 

# Setup Github command line tool
RUN \
	apt-get update \
	&& apt-key adv --keyserver keyserver.ubuntu.com --recv-key C99B11DEB97541F0 \
	&& apt-add-repository https://cli.github.com/packages \
	&& apt-get update \
	&& apt-get install -y \
		gh \
	&& rm -rf /var/lib/apt/lists/*

# Put things you want in your ~/.bashrc here
RUN \
	unset DEBIAN_FRONTEND \
	&& echo " \
alias gb='git branch -a'\n\
alias gcb='git checkout -b '\n\
alias gd='git diff HEAD'\n\
alias gg='git gui'\n\
alias gl='git log --pretty=oneline --graph'\n\
alias rh='cd $ROS_WS'\n\
alias gs='git status'\n\
alias code='code --user-data-dir /root/.visual_code/'\n\
alias STRESS='stress -c $(nproc) -m $(nproc) -i $(nproc) -d $(nproc)'\n\
source ${ROS_WS}/devel/setup.bash\n\
alias rvi='roslaunch ros-vision-interaction qt_sar_demo_interaction.launch'" >> ~/.bashrc

