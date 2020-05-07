#!/bin/bash
#
#  Copyright (C) 2014-2018 Dennis Stampfer, Matthias Lutz, Alex Lotz
#
#        Servicerobotik Ulm
#        University of Applied Sciences
#        Prittwitzstr. 10
#        D-89075 Ulm
#        Germany
#
#  Redistribution and use in source and binary forms, with or without modification, 
#  are permitted provided that the following conditions are met:
#  
#  1. Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#  
#  2. Redistributions in binary form must reproduce the above copyright notice, 
#     this list of conditions and the following disclaimer in the documentation 
#     and/or other materials provided with the distribution.
#  
#  3. Neither the name of the copyright holder nor the names of its contributors 
#     may be used to endorse or promote products derived from this software 
#     without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
#  OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# Dennis Stampfer 10/2014
# updates+compiles the virtual machine image to new ACE/smartsoft and toolchain version
#
# Matthias Lutz 10/2014
# install ACE/SmartSoft and dependencies to a clean Ubuntu 12.04 LTS install
#
# Dennis Stampfer 13.11.2014
# Error handling, bugfixes, Ace instead of Ace Tao, added ssh to packages list
#
# Dennis Stampfer 13.11.2014
# Verbose download for large (toolchain) file.
#
# Matthias Lutz 14.01.2016
# update locations of robotino repos
#
# Dennis Stampfer 15.1.2016
# Compatibility for raspberry-pi / arm architecture. More specific: Raspbian 8.0/jessie
#
# Matthias Lutz 00.12.2016
# Ununtu 16.04 as supported OS
#
# Dennis Stampfer 23.7.2018
# Adoption to v3-generation of SmartSoft World
#
# Dennis Stampfer, Alex Lotz 7.8.2018
# Update of Component Developer API way of installation
#
# Dennis Stampfer 9 2018
# Adoption to work on raspberry pi
#
# Dennis Stampfer 5.11.2018
# Fixing an issue with "source .profile" with custom prompts
#
# Dennis Stampfer 20.12.2018
# Temporarily deactivating toolchain installer since we are changing installation procedure
#
# Alex Lotz 23.10.2019
# Add check_sudo function and add latest toolchain installation commands
#
# Alex Lotz 12.12.2019
# Update Toolchain Installation to use version 3.12
#
# Matthias Lutz 07.05.2020
# Add ubuntu 20.04 support

# DO NOT ADD CODE ABOVE THIS LINE
# The following if is to test the update script.
if [ "$1" == "script-update-test" ]; then
	echo "ok"
	exit 0
fi
BCMD=$1 #fixes overwriting of prompt in .bashrc which we will source' later.
################################
# Insert code after here
################################

source ~/.profile

SCRIPT_DIR=`pwd`
SCRIPT_NAME=$0
SCRIPT_UPDATE_URL="https://github.com/Servicerobotics-Ulm/UtilityRepository/raw/master/smartsoft-install-updatescript.sh"

TOOLCHAIN_NAME="SmartMDSD-Toolchain"
TOOLCHAIN_VERSION="3.12"
TOOLCHAIN_URL="https://github.com/Servicerobotics-Ulm/SmartMDSD-Toolchain/releases/download/v$TOOLCHAIN_VERSION/SmartMDSD-Toolchain-v$TOOLCHAIN_VERSION.tar.gz"
TOOLCHAIN_LAUNCHER="$TOOLCHAIN_NAME.desktop"

COMMIT='$Id$'

echo "Update Script git=$COMMIT"

function abort() {
	echo 100 > /tmp/install-msg.log
	echo -e "\n\n### Aborted.\nYou can find a logfile in your current working directory:\n"
	pwd
	kill `cat /tmp/smartsoft-install-update.pid`
}

function askabort() {
	if zenity --width=400 --question --text="An error occurred (see log file). Abort update script?\n"; then
		abort
	fi
}

function progressbarinfo() {
	echo "# $1" > /tmp/install-msg.log
	echo -e "\n\n"
	echo "# $1"
	echo -e "\n\n"
}

# check if sudo is allowed and if necessary ask for password
function check_sudo() {
  local prompt

  # check for sudo rights without prompt
  prompt=$(sudo -nv 2>&1)
  if [ $? -eq 0 ]; then
    echo "has_sudo"
  elif echo $prompt | grep -q '^sudo:'; then
    PASSWD=$(zenity --title "sudo password" --password) || exit 1
    echo -e "$PASSWD\n" | sudo -Sv
    if [ $? -eq 0 ]; then
      echo "has_sudo"
    else
      abort
    fi
  else
    abort
  fi
}

if `grep --ignore-case precise /etc/os-release > /dev/null`; then 
	OS_PRECISE=true
fi

if `grep --ignore-case raspbian /etc/os-release > /dev/null`; then 
	OS_RASPBIAN=true
fi

if `grep --ignore-case xenial /etc/os-release > /dev/null`; then 
	OS_XENIAL=true
fi

if `grep --ignore-case focal /etc/os-release > /dev/null`; then 
	OS_FOCAL=true
fi
if `ping -c 1 i-zafh-03.srrc.informatik.hs-ulm.de &> /dev/null`; then 
	LOCATION_SRRC=true
fi





if ! [ -x "$(command -v xterm)" ]; then
	echo
	echo "ERROR: xterm not found. Install using 'sudo apt-get install xterm'"
	echo
	exit
fi


if ! [ -x "$(command -v zenity)" ]; then
	echo
	echo "ERROR: zenity not found. Install using 'sudo apt-get install zenity'"
	echo
	exit
fi


case "$BCMD" in

###############################################################################
# MENU
###############################################################################
menu)
	if [ "$OS_RASPBIAN" = true ]; then 
		zenity --info  --width=400  --text="Raspberry Pi was detected. Performing specific instructions for raspberry pi."
	fi

	if [ "$OS_XENIAL" = false -a "$OS_FOCAL" = false ]; then 
		zenity --info  --width=400 --text="Ubuntu 16.04 (Xenial) or 20.04 (Focal) was not detected. Please note that the officially supported plattform is a plain Ubuntu 16.04 and 20.04 installation."
	fi

	ACTION=$(zenity \
		--title "SmartSoft Install & Update Script" \
		--text "This is the automatic installation script for the SmartSoft World (v3-generation).\nThe default selection will install the SmartMDSD Toolchain with a full ACE/SmartSoft development environment.\n\nPlease select update actions to perform:\n\n* uses sudo: enter your password to the terminal window that pops up next." \
		--list --checklist \
		--height=350 \
		--width=430 \
		--column="" --column=Action --column=Description \
		--hide-column=2 --print-column=2 --hide-header \
		--separator="|" \
		true package-upgrade "1) Upgrade system packages*" \
		true toolchain-update "2) Update/Install SmartMDSD Toolchain to latest version" \
		true menu-install "3) Install ACE/SmartSoft Development Environment and dependencies on a clean system*" \
		true repo-up-smartsoft "4) Update ACE/SmartSoft Development Environment (updates repositories)" \
		true build-smartsoft "5) Build/Compile ACE/SmartSoft Development Environment" \
#		false svn-up-robotino "X) Update Robotino repositories" \
#		false build-robotino "X) Build/Compile Robotino ACE/SmartSoft Components" \
	) || exit 1

	CMD=""
	IFS='|';
	for A in $ACTION; do
		CMD="$CMD bash $SCRIPT_NAME $A || askabort;"
	done
	LOGFILE=`basename $0`.`date +"%Y%m%d%H%M"`.log
	xterm -title "Updating..." -hold -e "exec > >(tee $LOGFILE); exec 2>&1; echo '### Update script start (git=$COMMIT)'; date; echo 'Logfile: $LOGFILE'; $CMD echo;echo;echo '### Update script finished. Logfile: $LOGFILE';echo 100 > /tmp/install-msg.log;echo;echo;rm /tmp/smartsoft-install-update.pid; date" &
	echo $! > /tmp/smartsoft-install-update.pid

	progressbarinfo "Starting ..."
	tail -f /tmp/install-msg.log | zenity --progress --title="Installing ..." --auto-close --text="Starting ..." --pulsate --width=500 &

	#echo -e "icon:info\ntooltip:Update script finished."|zenity --notification --listen &

	exit 0
;;

###############################################################################
# MENU INSTALL
###############################################################################
menu-install)
	progressbarinfo "Launching installation menu for ACE/SmartSoft"

	zenity --question --width=500 --text="<b>ATTENTION</b>\n The script is about to install ACE/SmartSoft and dependency packages on this system.\n<b>Only use this function on a clean installation of Ubuntu 16.04 or 20.04.</b> Some of the following steps may not be execute twice without undoing them before.\n\n(support for Raspbian 8.0/Jessie and other distributions is experimental)\n\nDo you want to proceed?" || abort 

	ACTION=$(zenity \
		--title "Install ACE/SmartSoft and dependencies on a clean system" \
		--text "About to install a development environment.\nPlease select update actions to perform:\n" \
		--list --checklist \
		--height=270 \
		--width=620 \
		--column="" --column=Action --column=Description \
		--hide-column=2 --print-column=2 --hide-header \
		--separator="|" \
		true package-install "1.1) Install system packages required for ACE/SmartSoft" \
		true ace-source-install "1.2) Install ACE from source or package" \
		true repo-co-smartsoft "1.3) Checkout ACE/SmartSoft repository and set environment variables" \
        	false package-internal-install "1.4) Install additional generic packages (optional)" \
#		false package-install-robotino "X) Install packages for robotino robot" \
#		false svn-co-robotino "X) Checkout ACE/SmartSoft repository for robotino robot" \
	) || abort


	IFS='|';
	for A in $ACTION; do
		bash $SCRIPT_NAME $A || askabort
	done
	echo
	echo
	echo '### Install script finished.'
	progressbarinfo "Finished"
	echo
	echo

	#echo -e "icon:info\ntooltip:Installation script finished." | zenity --notification --listen &

	exit 0
;;

###############################################################################
ace-source-install)
	# become root
	if [ "$(id -u)" != "0" ]; then
		sudo bash $SCRIPT_NAME $1
		exit 0
	fi

 	# FOCAL (20.04 Packages)
        if [ "$OS_FOCAL" = true ]; then
		progressbarinfo "Installing ACE from packages..."
                apt-get -y --force-yes install libace-dev || askabort
		exit 0
        fi

	progressbarinfo "Running ACE source install (will take some time)"

	sleep 2

	wget -nv https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework/raw/master/INSTALL-ACE-6.0.2.sh -O /tmp/INSTALL-ACE-6.0.2.sh || askabort
	chmod +x /tmp/INSTALL-ACE-6.0.2.sh || askabort
	/tmp/INSTALL-ACE-6.0.2.sh /opt || askabort

        echo "/opt/ACE_wrappers/lib" > /etc/ld.so.conf.d/ace.conf || askabort
	ldconfig || askabort
	exit 0
;;

###############################################################################
package-install)
	# become root
	if [ "$(id -u)" != "0" ]; then
		sudo bash $SCRIPT_NAME $1
		exit 0
	fi


	progressbarinfo "Running package install ..."

	sleep 2
	progressbarinfo "Running apt-get update, upgrade ..."
	apt-get -y --force-yes update || askabort
	apt-get -y --force-yes upgrade || askabort

	progressbarinfo "Installing packages ..."
	# General packages:
	apt-get -y --force-yes install ssh-askpass git flex bison htop tree cmake cmake-curses-gui subversion sbcl doxygen \
 meld expect wmctrl libopencv-dev libboost-all-dev libftdi-dev \
 build-essential pkg-config freeglut3-dev zlib1g-dev zlibc libusb-1.0-0-dev libdc1394-22-dev libavformat-dev libswscale-dev \
 lib3ds-dev libjpeg-dev libgtest-dev libeigen3-dev libglew-dev vim libxml2-dev libxml++2.6-dev ssh sshfs xterm libjansson-dev || askabort




	
	progressbarinfo "Installing OS-specific packages ..."

	# 12.04 packages
	if [ "$OS_PRECISE" = true ]; then
		apt-get -y --force-yes install libwxgtk2.8-dev openjdk-6-jre libtbb-dev libmrpt-dev libcv-dev libcvaux-dev libhighgui-dev || askabort
	fi

	# Other packages to install - except for raspberry pi:
	if [ "$OS_RASPBIAN" = true ]; then 
		apt-get -y --force-yes install libwxgtk2.8-dev libmrpt-dev libcv-dev libcvaux-dev libhighgui-dev || askabort
	fi

	# Xenial (16.04 Packages)
	if [ "$OS_XENIAL" = true ]; then
		apt-get -y --force-yes install openjdk-8-jre libtbb-dev libmrpt-dev libcv-dev libcvaux-dev libhighgui-dev || askabort
	fi


 	# Xenial (20.04 Packages)
        if [ "$OS_FOCAL" = true ]; then
		progressbarinfo "Installing mrpt ..."
		# Install mrpt from ppa since not available in 20.04
		sudo add-apt-repository ppa:joseluisblancoc/mrpt
		sudo apt-get update
		sudo apt-get install libmrpt-dev mrpt-apps

                apt-get -y --force-yes install openjdk-11-jre libtbb-dev || askabort
        fi


	exit 0
;;

###############################################################################
package-internal-install)
	# become root
	if [ "$(id -u)" != "0" ]; then
		sudo bash $SCRIPT_NAME $1
		exit 0
	fi

	progressbarinfo	"Running internal package install ..."
	sleep 2
	apt-get update || askabort

	apt-get -y --force-yes update || askabort
	apt-get -y --force-yes upgrade || askabort

	# Other packages to install - except for raspberry pi:
	#if [ ! "$OS_RASPBIAN" = true ]; then 
	#	apt-get -y --force-yes install libwxgtk2.8-dev || askabort
	#fi

	# Xenial (16.04 Packages)
	if [ "$OS_XENIAL" = true ]; then
		apt-get -y --force-yes install gparted gnome-session-flashback gconf-editor cmake-curses-gui inkscape gimp mplayer qiv nautilus-compare git || askabort
	fi

	exit 0
;;
###############################################################################

package-install-robotino)
	zenity --info --width=400 --text="You selected to install packages for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
	abort
	# become root
#	if [ "$(id -u)" != "0" ]; then
#		sudo bash $SCRIPT_NAME $1
#		exit 0
#	fi
#
#	echo -e "\n\n\n### Running package install ...\n\n\n"
#	sleep 2
#	
#	CODENAME=`lsb_release -cs`
#
#       echo "deb http://packages.openrobotino.org/$CODENAME $CODENAME main" > /etc/apt/sources.list.d/openrobotino.list
#	apt-get update || askabort
#
#	apt-get -y --force-yes install robotino-api2 rec-rpc libqt4-dev robotino-common || askabort
#	exit 0
;;

###############################################################################
package-upgrade)
	# become root
	if [ "$(id -u)" != "0" ]; then
		sudo bash $SCRIPT_NAME $1
		exit 0
	fi

	progressbarinfo "Updating system packages ... (apt-get update)"
	sleep 2
	apt-get -y --force-yes update || askabort

	progressbarinfo "Updating system packages ... (apt-get upgrade)"
	apt-get -y --force-yes upgrade || askabort
	exit 0
;;

###############################################################################
# checkout Public repository
repo-co-smartsoft)
	# check if we are in the lab and then ask wether to continue to install external stuff
	# or quit and continue with internal stuff
	if [ "$LOCATION_SRRC" = true ]; then
		if zenity --question --width=400 --text="It appears that you are installing from within the SRRC laboratory.\n\nDo you want to use the internal repositories instead of the public repositories?\n"; then
			bash $SCRIPT_NAME repo-co-smartsoft-internal
			exit 0
		fi
	fi

	progressbarinfo "Cloning repositories"

	sleep 2

	mkdir -p ~/SOFTWARE/smartsoft-ace-mdsd-v3/repos || askabort
	ln -s ~/SOFTWARE/smartsoft-ace-mdsd-v3 ~/SOFTWARE/smartsoft || askabort



 	# Xenial (20.04 Packages)
        if [ "$OS_FOCAL" = false ]; then
		echo "export ACE_ROOT=/opt/ACE_wrappers" >> ~/.profile
	fi
	echo "export SMART_ROOT_ACE=\$HOME/SOFTWARE/smartsoft" >> ~/.profile
	echo "export SMART_PACKAGE_PATH=\$SMART_ROOT_ACE/repos" >> ~/.profile
	#echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\$SMART_ROOT_ACE/lib" >> ~/.profile
	echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$SMART_ROOT_ACE/lib" >> ~/.bashrc

	source ~/.profile 

	cd ~/SOFTWARE/smartsoft-ace-mdsd-v3/repos || askabort

	progressbarinfo "Cloning repositories SmartSoftComponentDeveloperAPIcpp.git"
	git clone https://github.com/Servicerobotics-Ulm/SmartSoftComponentDeveloperAPIcpp.git || askabort
	progressbarinfo "Cloning repositories AceSmartSoftFramework.git"
	git clone https://github.com/Servicerobotics-Ulm/AceSmartSoftFramework.git || askabort
	progressbarinfo "Cloning repositories UtilityRepository.git"
	git clone https://github.com/Servicerobotics-Ulm/UtilityRepository.git || askabort
	progressbarinfo "Cloning repositories DataRepository.git"
	git clone https://github.com/Servicerobotics-Ulm/DataRepository.git || askabort
	progressbarinfo "Cloning repositories DomainModelsRepositories.git"
	git clone https://github.com/Servicerobotics-Ulm/DomainModelsRepositories.git || askabort
	progressbarinfo "Cloning repositories ComponentRepository.git"
	git clone https://github.com/Servicerobotics-Ulm/ComponentRepository.git || askabort
	progressbarinfo "Cloning repositories SystemRepository.git"
	git clone https://github.com/Servicerobotics-Ulm/SystemRepository.git || askabort


	zenity --info --width=400 --text="Environment settings in .profile have been changed. In order to use them, \ndo one of the following after the installation script finished:\n\n- Restart your computer\n- Logout/Login again\n- Execute 'source ~/.profile'"  --height=100

	exit 0
;;

###############################################################################
# checkout SRRC-Internal repository
repo-co-smartsoft-internal)
	echo -e "\n\n\n### Running repo checkout (SRRC-INTERNAL REPOSITORIES) ...\n\n\n"
	sleep 2

	progressbarinfo "Cloning repositories"

	mkdir -p ~/SOFTWARE/smartsoft-ace-mdsd-v3/repos || askabort
	ln -s ~/SOFTWARE/smartsoft-ace-mdsd-v3 ~/SOFTWARE/smartsoft || askabort

	echo "export ACE_ROOT=/opt/ACE_wrappers" >> ~/.profile
	echo "export SMART_ROOT_ACE=\$HOME/SOFTWARE/smartsoft" >> ~/.profile
	echo "export SMART_PACKAGE_PATH=\$SMART_ROOT_ACE/repos" >> ~/.profile
	echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\$SMART_ROOT_ACE/lib" >> ~/.profile

	source ~/.profile 

	cd ~/SOFTWARE/smartsoft-ace-mdsd-v3/repos || askabort

	if ! [ -d "/mnt/ssh/robo/repositories/smartSoftDev_v3/" ]; then
		zenity --info --width=400 --text="Error: /mnt/ssh/robo/repositories/smartSoftDev_v3/ is not accessible.\nPlease mount it before continuing.\n(you can keep this window open / the script active while doing so...)"
	fi

	progressbarinfo "Cloning repositories SmartSoftComponentDeveloperAPIcpp.git.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/SmartSoftComponentDeveloperAPIcpp.git || askabort

	progressbarinfo "Cloning repositories AceSmartSoftFramework.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/AceSmartSoftFramework.git || askabort

	progressbarinfo "Cloning repositories UtilityRepository.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/UtilityRepository.git || askabort

	progressbarinfo "Cloning repositories DataRepository.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/DataRepository.git || askabort

	progressbarinfo "Cloning repositories DomainModelsRepositories.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/DomainModelsRepositories.git || askabort

	progressbarinfo "Cloning repositories ComponentRepository.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/ComponentRepository.git || askabort

	progressbarinfo "Cloning repositories SystemRepository.git"
	git clone /mnt/ssh/robo/repositories/smartSoftDev_v3/SystemRepository.git || askabort

	zenity --info --width=400 --text="Environment settings in .profile have been changed. In order to use them, do one of the following:\n\n- Restart your computer\n- Logout/Login again\n- Execute 'source ~/.profile'"

	exit 0

;;

###############################################################################
repo-up-smartsoft)
	echo -e "\n\n\n### Running ACE/SmartSoft repo update ...\n\n\n"
	progressbarinfo "About to update repositories ..."
	sleep 2

	if zenity --question --width=400 --text="The installation script is about to update the repositories.\nThis will <b>overwrite all your modifications</b> that you did to the repositories in \$SMART_ROOT_ACE/repos/.\n\nDo you want to proceed?\n\nIt is safe to do so in case you did not modify SmartMDSD Toolchain projects or don't need the modifications anymore.\nIf you choose not to update, please do a 'git pull' for the repositories yourself."; then
		echo -e "\n\n\n# Continuing with repo update.\n\n\n"
	else
		echo -e "\n\n\n# Not running repo update.\n\n\n"
		exit 0
	fi

	progressbarinfo "Running ACE/SmartSoft repo update SmartSoftComponentDeveloperAPIcpp"
	cd $SMART_ROOT_ACE/repos/SmartSoftComponentDeveloperAPIcpp || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update AceSmartSoftFramework"
	cd $SMART_ROOT_ACE/repos/AceSmartSoftFramework || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update UtilityRepository"
	cd $SMART_ROOT_ACE/repos/UtilityRepository || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update DataRepository"
	cd $SMART_ROOT_ACE/repos/DataRepository || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update DomainModelsRepositories"
	cd $SMART_ROOT_ACE/repos/DomainModelsRepositories || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update ComponentRepository"
	cd $SMART_ROOT_ACE/repos/ComponentRepository || askabort
	git reset --hard HEAD
	git pull || askabort

	progressbarinfo "Running ACE/SmartSoft repo update SystemRepository"
	cd $SMART_ROOT_ACE/repos/SystemRepository || askabort
	git reset --hard HEAD
	git pull || askabort

	exit 0
;;

###############################################################################
build-smartsoft)
	echo -e "\n\n\n### Running Build ACE/SmartSoft ...\n\n\n"
	sleep 2

	progressbarinfo "Running Build ACE/SmartSoft SmartSoftComponentDeveloperAPIcpp ..."
	cd $SMART_ROOT_ACE/repos/SmartSoftComponentDeveloperAPIcpp || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make install || askabort

	progressbarinfo "Running Build ACE/SmartSoft Kernel ..."
	# warkaround for the case when the kernel is not built automatically as external dependency
	cd $SMART_ROOT_ACE/repos/AceSmartSoftFramework || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make install || askabort

	progressbarinfo "Running Build Utilities"
	cd $SMART_ROOT_ACE/repos/UtilityRepository || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make || askabort

	progressbarinfo "Running Build DomainModels"
	cd $SMART_ROOT_ACE/repos/DomainModelsRepositories || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make || askabort

	progressbarinfo "Running Build Components"
	cd $SMART_ROOT_ACE/repos/ComponentRepository || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make || askabort

	exit 0
;;

###############################################################################
svn-co-robotino)
	zenity --info --width=400 --text="You selected to svn-co for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
	abort
#	echo -e "\n\n\n### Running Robotino ACE/SmartSoft SVN checkout ...\n\n\n"
#	sleep 2
#
#	mkdir -p ~/SOFTWARE/smartsoft_robotino_components
#
#	svn co svn://svn.rec.de/openrobotino/smartsoft/trunk/components ~/SOFTWARE/smartsoft_robotino_components || askabort
#	exit 0
;;

###############################################################################
svn-up-robotino)
	zenity --info --width=400 --text="You selected to svn-up for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
#	echo -e "\n\n\n### Running Robotino SVN update ...\n\n\n"
#	sleep 2
#
#	cd ~/SOFTWARE/smartsoft_robotino_components
#	svn up || askabort
#	exit 0
;;

###############################################################################
build-robotino)
	zenity --info --width=400 --text="You selected to build-robotino for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
#	echo -e "\n\n\n### Running Build robotino ...\n\n\n"
#	sleep 2
#
#	cd ~/SOFTWARE/smartsoft_robotino_components || askabort
#	for I in Smart*; do
#		cd ~/SOFTWARE/smartsoft_robotino_components/$I
#		mkdir build
#		cd build
#		cmake ..
#		make || askabort
#	done
#	exit 0
;;

###############################################################################
toolchain-update)
	progressbarinfo "Running toolchain installation ..."
	# check if OpenJDK 8 is installed (autoinstall it if needed)
	if [[ $(java -version 2>&1) == "openjdk version \"1.8"* ]]; then
		echo "-- found OpenJDK 1.8"
	else
		progressbarinfo "Installing dependency OpenJDK 8 ..."
		check_sudo
		sudo apt install -y openjdk-8-jre || askabort
	fi

	progressbarinfo "Downloading the SmartMDSD Toolchain archive from: $TOOLCHAIN_URL"
	cd $HOME/SOFTWARE
	wget -N $TOOLCHAIN_URL || askabort
	wget --progress=dot:mega --content-disposition $TOOLCHAIN_URL || askabort

	progressbarinfo "Extracting the SmartMDSD Toolchain archive $TOOLCHAIN_NAME-v$TOOLCHAIN_VERSION.tar.gz" 
	tar -xzf $TOOLCHAIN_NAME-v$TOOLCHAIN_VERSION.tar.gz || askabort

	# create a desktop launcher
	echo "#!/usr/bin/xdg-open" > /tmp/$TOOLCHAIN_LAUNCHER
	echo "[Desktop Entry]" >> /tmp/$TOOLCHAIN_LAUNCHER
	echo "Name=SmartMDSD Toolchain v$TOOLCHAIN_VERSION" >> /tmp/$TOOLCHAIN_LAUNCHER	
	echo "Version=$TOOLCHAIN_VERSION" >> /tmp/$TOOLCHAIN_LAUNCHER
	
	cd $TOOLCHAIN_NAME-v$TOOLCHAIN_VERSION
	echo "Exec=$PWD/eclipse" >> /tmp/$TOOLCHAIN_LAUNCHER

	cd plugins/org.smartmdsd.branding*
	cd icons
	echo "Icon=$PWD/logo64.png" >> /tmp/$TOOLCHAIN_LAUNCHER

	echo "Terminal=false" >> /tmp/$TOOLCHAIN_LAUNCHER
	echo "Type=Application" >> /tmp/$TOOLCHAIN_LAUNCHER
	echo "Categories=Development;" >> /tmp/$TOOLCHAIN_LAUNCHER

	cd /tmp
	chmod +x $TOOLCHAIN_LAUNCHER
	cp $TOOLCHAIN_LAUNCHER $HOME/.local/share/applications/
	cp $TOOLCHAIN_LAUNCHER $(xdg-user-dir DESKTOP)

#	if ! [ -x "$(command -v gio)" ]; then
#		progressbarinfo "Installing dependency libglib2.0-bin (to use the GIO tool) ..."
#		check_sudo
#		sudo apt-get install -y libglib2.0-bin
#	fi
#	if [ -x "$(command -v gio)" ]; then
#		gio set $HOME/Desktop/$TOOLCHAIN_LAUNCHER "metadata::trusted" yes
#	fi

	exit 0
	

#	TC_DOWNLOAD=`tempfile`
#	progressbarinfo "Downloading SmartMDSD Toolchain ..."
#	wget --progress=dot:mega --content-disposition $TOOLCHAIN_LATEST_URL -O $TC_DOWNLOAD || askabort
#
#	progressbarinfo "Setting up SmartMDSD Toolchain ..."
#
#	mv ~/SOFTWARE/SmartMDSD_Toolchain.latest ~/SOFTWARE/SmartMDSD_Toolchain.`date +%Y-%m-%d` 
#	mkdir -p ~/SOFTWARE/SmartMDSD_Toolchain.latest 
#	cd ~/SOFTWARE/SmartMDSD_Toolchain.latest || askabort
#
#	#TCNAME=$(ls * | sed "s/\.tar.*//g" | sed "s/\.bz2.*//g")
#	tar xf $TC_DOWNLOAD || askabort
#	EXECUTABLE=`pwd`/$(find ./ -name eclipse -type f)
#	ICON=`pwd`/$(find ./ -name icon.xpm -type f)
#
#
#	echo "Toolchain executable is: $EXECUTABLE"
#	
#	echo -e "\n# Setting up workspace path ...\n"
#	IDEPREFS=$(find ./ -name org.eclipse.ui.ide.prefs)
#	echo "MAX_RECENT_WORKSPACES=5
#RECENT_WORKSPACES=$HOME/workspaces/SmartMDSD-Toolchain
#RECENT_WORKSPACES_PROTOCOL=3
#SHOW_WORKSPACE_SELECTION_DIALOG=true
#eclipse.preferences.version=1
#" > $IDEPREFS
#
#	progressbarinfo "Creating desktop starter ..."
#echo "[Desktop Entry]
#Encoding=UTF-8
#Version=1.0
#Name=SmartMDSD Toolchain
#Comment=Starts the latest version of the SmartMDSD Toolchain
#Type=Application
#Exec=$EXECUTABLE
#Icon=$ICON
#" > ~/Desktop/SmartMDSDToolchain.desktop
#	chmod +x ~/Desktop/SmartMDSDToolchain.desktop
#	exit 0
;;


###############################################################################
# Update the virtual machine. Only run from within the vm!
###############################################################################
vm-update)

	if [ "$(hostname)" != "smartsoft-vm" ]; then
		echo "Virtual machine was not detected."
		zenity --question --width=400 --text="<b>Warning</b>: Virtual machine was not detected.\nOnly run this action from within the virtual machine.\n\nDo you want to proceed at your own risk?" || abort
	fi

	if zenity --question --width=400 --text="This installation/update script has an updater included.\nDo you want to update this script before installing it?\n\nWill update script from:\n$SCRIPT_UPDATE_URL\n"; then
		bash $SCRIPT_NAME script-update
		exit 0
	else
		progressbarinfo "Not updating the script before running it."
	fi

	progressbarinfo "Starting ..."
	tail -f /tmp/install-msg.log | zenity --progress --title="Installing ..." --auto-close --text="Starting ..." --pulsate --width=500 &

	ACTION="toolchain-update|repo-up-smartsoft|build-smartsoft|vm-update-compile"

	CMD=""
	IFS='|';
	for A in $ACTION; do
		CMD="$CMD bash $SCRIPT_NAME $A || askabort;"
	done
	LOGFILE=`basename $0`.`date +"%Y%m%d%H%M"`.log
	xterm -title "Updating..." -hold -e "exec > >(tee $LOGFILE); exec 2>&1; echo '### Update script start (git=$COMMIT)'; date; echo 'Logfile: $LOGFILE'; $CMD echo;echo;echo '### Update script finished. Logfile: $LOGFILE';echo 100 > /tmp/install-msg.log;echo;echo;rm /tmp/smartsoft-install-update.pid; date" &

	echo $! > /tmp/smartsoft-install-update.pid

	exit 0
;;

# compiles vm specific components
vm-update-compile)

	progressbarinfo "Compiling vm-specific components ..."
	sleep 2

	progressbarinfo "Compiling Player Stage Component"
	cd $SMART_ROOT_ACE/repos/ComponentRepository/ComponentPlayerStageSimulator/smartsoft/ || askabort
	mkdir build
	cd build || askabort
	cmake .. || askabort
	make || askabort

	progressbarinfo "Compiling Gazebo Simulator Component"
	cd $SMART_ROOT_ACE/repos/ComponentRepository/SmartGazeboBaseServer/smartsoft/ || askabort
	mkdir build
	cd build || askabort
	cmake .. || askabort
	make || askabort

	progressbarinfo "Compiling ComponentLaserObstacleAvoid"
	cd $SMART_ROOT_ACE/repos/ComponentRepository/ComponentLaserObstacleAvoid/smartsoft/ || askabort
	mkdir build
	cd build || askabort
	cmake .. || askabort
	make || askabort

	progressbarinfo "Compiling vm-specific components ... Done."
	sleep 1

	exit 0
;;


###############################################################################
# Update the installation script
###############################################################################
script-update)
	progressbarinfo "Updating the script before starting it ..."
	T=`tempfile`
	echo "Tempfile: $T"
	
	wget "$SCRIPT_UPDATE_URL" -O $T

	if [ "$(file --mime-type -b $T)" != "text/x-shellscript" ]; then
		zenity --info --text="Error updating the script."
		echo -e "\n # This is not a shell script."
		exit
	fi
	echo -e "\n # File is a shell script."

	if [ "$(bash $T script-update-test)" != "ok" ]; then
		zenity --info --text="Error updating the script."
		echo -e "\n # bash $T script-update-test : returned not OK"
		exit
	fi
	echo -e "\n # Test OK"

	mv $T $SCRIPT_NAME

	zenity --info --width=400 --text="Update finished. Please restart the\ninstallation script (and choose not to update)."

	exit 0
;;



###############################################################################
# The usual entry point of this script. We use this to determine the
# default action. No extra code should go here.
###############################################################################
start)
	bash $SCRIPT_NAME menu
;;




###############################################################################
*)
	if zenity --question --text="This installation and update script has an updater included.\nDo you want to update this script before continuing?\n\nUpdate location:\n$SCRIPT_UPDATE_URL\n"; then
		bash $SCRIPT_NAME script-update
		exit 0
	else
		echo -e "\n\n\n# Not updating the script before running it.\n\n\n"
	fi

	bash $SCRIPT_NAME start
;;


esac


