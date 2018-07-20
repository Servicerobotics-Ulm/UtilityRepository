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
# Dennis Stampfer 19.7.2018
# Adoption of skeleton to v3-generation of SmartSoft World
#
#
#
#
#
#
# DO NOT ADD CODE ABOVE THIS LINE
# The following if is to test the update script.
if [ "$1" == "script-update-test" ]; then
	echo "ok"
	exit 0
fi
################################
# Insert code after here
################################

source ~/.profile

SCRIPT_DIR=`pwd`
SCRIPT_NAME=$0
SCRIPT_UPDATE_URL="https://github.com/Servicerobotics-Ulm/UtilityRepository/raw/master/smartsoft-install-updatescript.sh"
TOOLCHAIN_LATEST_URL="https://web2.servicerobotik-ulm.de/files/SmartMDSD_Toolchain/latest.tar.gz"
COMMIT='$Id$'

echo "Update Script git=$COMMIT"

function abort() {
	echo -e "\n\n### Aborted.\nYou can find a logfile in your current working directory:\n"
	pwd
	kill `cat /tmp/smartsoft-install-update.pid`
}

function askabort() {
	if zenity --question --text="An error occurred (see log file). Abort update script?\n"; then
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

if `ping -c 1 i-zafh-03.srrc.informatik.hs-ulm.de &> /dev/null`; then 
	LOCATION_SRRC=true
fi



case "$1" in

###############################################################################
# MENU
###############################################################################
menu)
	if [ "$OS_RASPBIAN" = true ]; then 
		zenity --info --text="Raspberry Pi was detected. Performing specific instructions for raspberry pi. THIS IS YET UNTESTED AND EXPERIMENTAL."
	fi

	if [ "$OS_XENIAL" = false ]; then 
		zenity --info --text="Ubuntu 16.04 (Xenial) was not detected. Please note that the officially supported plattform is a plain Ubuntu 16.04 installation."
	fi

	ACTION=$(zenity \
		--title "SmartSoft Install & Update Script" \
		--text "This is the automatic installation script for the SmartSoft World. Depending\non your selection, it will install the SmartMDSD Toolchain with a full development environment.\n\nPlease select update actions to perform:\n\n* action uses sudo" \
		--list --checklist \
		--height=350 \
		--width=430 \
		--column="" --column=Action --column=Description \
		--hide-column=2 --print-column=2 --hide-header \
		--separator="|" \
		true package-upgrade "1) Upgrade system packages*" \
		false menu-install "2) Install ACE/SmartSoft and dependencies on a clean system*" \
		true repo-up-smartsoft "3) Update ACE/SmartSoft SVN" \
		true build-smartsoft "4) Build/Compile ACE/SmartSoft" \
		false svn-up-robotino "5) Update Robotino SVN" \
		false build-robotino "6) Build/Compile Robotino ACE/SmartSoft Components" \
		true toolchain-update "7) Update/Install SmartMDSD Toolchain to latest version" \
	) || exit 1

	CMD=""
	IFS='|';
	for A in $ACTION; do
		CMD="$CMD bash $SCRIPT_NAME $A || askabort;"
	done
	LOGFILE=`basename $0`.`date +"%Y%m%d%H%M"`.log
	xterm -title "Updating..." -hold -e "exec > >(tee $LOGFILE); exec 2>&1; echo '### Update script start (git=$COMMIT)'; date; echo 'Logfile: $LOGFILE'; $CMD echo;echo;echo '### Update script finished. Logfile: $LOGFILE';echo;echo;rm /tmp/smartsoft-install-update.pid; date" &
	echo $! > /tmp/smartsoft-install-update.pid

	#echo -e "icon:info\ntooltip:Update script finished."|zenity --notification --listen &

	exit 0
;;

###############################################################################
# MENU INSTALL
###############################################################################
menu-install)
	zenity --question --text="ATTENTION:\n The script is about to install ACE/SmartSoft and dependency packages on this system. Only use this function on a clean installation of Ubuntu 16.04. Some of the following steps may not be execute twice without undoing them before.\n\nProceed?\n\n* Experimental support for Raspbian 8.0/Jessie available" || abort 

	ACTION=$(zenity \
		--title "Install ACE/SmartSoft and dependencies on a clean system" \
		--text "Please select update actions to perform:\n" \
		--list --checklist \
		--height=270 \
		--width=420 \
		--column="" --column=Action --column=Description \
		--hide-column=2 --print-column=2 --hide-header \
		--separator="|" \
		false package-install "1.1) Install system packages required for ACE/SmartSoft" \
		false ace-source-install "1.2) Install ACE from source" \
		false repo-co-smartsoft "1.3) Checkout ACE/SmartSoft repository and set environment variables" \
		false package-install-robotino "1.4) Install packages for robotino robot" \
		false svn-co-robotino "1.5) Checkout ACE/SmartSoft repository for robotino robot" \
        	false package-internal-install "1.6) Install additional generic packages (optional)"\
	) || abort

	IFS='|';
	for A in $ACTION; do
		bash $SCRIPT_NAME $A || askabort
	done
	echo
	echo
	echo '### Install script finished.'
	echo
	echo

	echo -e "icon:info\ntooltip:Installation script finished." | zenity --notification --listen &

	exit 0
;;

###############################################################################
# TODO ALEX
ace-source-install)
	# become root
	if [ "$(id -u)" != "0" ]; then
		sudo bash $SCRIPT_NAME $1
		exit 0
	fi

	echo -e "\n\n\n### Running ACE source install (will take some time) ...\n\n\n"
	sleep 2

	wget -nv http://sourceforge.net/p/smartsoft-ace/code/HEAD/tree/trunk/INSTALL-ACE-6.0.2.sh?format=raw -O /tmp/INSTALL-ACE-6.0.2.sh || askabort
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

	echo -e "\n\n\n### Running package install ...\n\n\n"
	sleep 2
	apt-get update || askabort

	apt-get -y --force-yes update || askabort
	apt-get -y --force-yes upgrade || askabort

	# General packages:
	apt-get -y --force-yes install flex bison htop tree cmake cmake-curses-gui subversion sbcl doxygen \
 meld expect wmctrl libopencv-dev libboost-all-dev libftdi-dev libcv-dev libcvaux-dev libhighgui-dev \
 build-essential pkg-config freeglut3-dev zlib1g-dev zlibc libusb-1.0-0-dev libdc1394-22-dev libavformat-dev libswscale-dev \
 lib3ds-dev libjpeg-dev libgtest-dev libeigen3-dev libglew-dev vim vim-gnome libxml2-dev libxml++2.6-dev libmrpt-dev ssh sshfs xterm libjansson-dev || askabort
	
	# 12.04 packages
	if [ "$OS_PRECISE" = true ]; then
		apt-get -y --force-yes install libwxgtk2.8-dev openjdk-6-jre libtbb-dev || askabort
	fi

	# Other packages to install - except for raspberry pi:
	if [ "$OS_RASPBIAN" = true ]; then 
		apt-get -y --force-yes install libwxgtk2.8-dev || askabort
	fi

	# Xenial (16.04 Packages)
	if [ "$OS_XENIAL" = true ]; then
		apt-get -y --force-yes install openjdk-8-jre libtbb-dev || askabort
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

	echo -e "\n\n\n### Running package install ...\n\n\n"
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
	zenity --info --text="You selected to install packages for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
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

	echo -e "\n\n\n### Running package upgrade ...\n\n\n"
	sleep 2
	apt-get update || askabort

	apt-get -y --force-yes update || askabort
	apt-get -y --force-yes upgrade || askabort
	exit 0
;;

###############################################################################
# checkout Public repository
# TODO alex
repo-co-smartsoft)
	# check if we are in the lab and then ask wether to continue to install external stuff
	# or quit and continue with internal stuff
	if [ $LOCATION_SRRC = true ]; then
		if zenity --question --text="It appears that you are installing from within the SRRC laboratory.\n\nDo you want to use the internal repositories instead of the public repositories?\n"; then
			bash $SCRIPT_NAME repo-co-smartsoft-internal
			exit 0
		fi
	fi

	zenity --info --text="will now install PUBLIC STUFF"
	exit 0

	echo -e "\n\n\n### Running repo checkout ...\n\n\n"
	sleep 2

	mkdir -p ~/SOFTWARE/smartsoft-ace-mdsd-v2
	ln -s ~/SOFTWARE/smartsoft-ace-mdsd-v2 ~/SOFTWARE/smartsoft || askabort

	echo "export ACE_ROOT=/opt/ACE_wrappers" >> ~/.profile
	echo "export SMART_ROOT_ACE=\$HOME/SOFTWARE/smartsoft" >> ~/.profile
	echo "export SMART_PACKAGE_PATH=\$SMART_ROOT_ACE/src" >> ~/.profile
	echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\$SMART_ROOT_ACE/lib" >> ~/.profile

	source ~/.profile 

	svn co http://svn.code.sf.net/p/smartsoft-ace/code/trunk/ ~/SOFTWARE/smartsoft-ace-mdsd-v2 || askabort
	
	zenity --info --text="Environment settings in .profile have been changed. In order to use them, do one of the following:\n\n- Restart your computer\n- Logout/Login again\n- Execute 'source ~/.profile'"  --height=100

	exit 0
;;

###############################################################################
# checkout SRRC-Internal repository
# todo alex
repo-co-smartsoft-internal)
	echo -e "\n\n\n### Running repo checkout (SRRC-INTERNAL REPOSITORIES) ...\n\n\n"
	sleep 2

	zenity --info --text="will now install INTERNAL"

;;

###############################################################################
# TODO update alex
repo-up-smartsoft)
	echo -e "\n\n\n### Running ACE/SmartSoft SVN update ...\n\n\n"
	sleep 2

	cd $SMART_ROOT_ACE || askabort
	svn up || askabort
	exit 0
;;

###############################################################################
# TODO alex
build-smartsoft)
	echo -e "\n\n\n### Running Build ACE/SmartSoft ...\n\n\n"
	sleep 2

	echo -e "\n\n\n### Running Build ACE/SmartSoft Kernel ...\n\n\n"
	# warkaround for the case when the kernel is not built automatically as external dependency
	cd $SMART_ROOT_ACE/src/smartSoftKernel || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make install || askabort

	echo -e "\n\n\n### Running Build ACE/SmartSoft Global-Build ...\n\n\n"
	cd $SMART_ROOT_ACE || askabort
	mkdir build
	cd build || askabort
	cmake ..
	make || askabort
	exit 0
;;

###############################################################################
svn-co-robotino)
	zenity --info --text="You selected to svn-co for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
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
	zenity --info --text="You selected to svn-up for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
#	echo -e "\n\n\n### Running Robotino SVN update ...\n\n\n"
#	sleep 2
#
#	cd ~/SOFTWARE/smartsoft_robotino_components
#	svn up || askabort
#	exit 0
;;

###############################################################################
build-robotino)
	zenity --info --text="You selected to build-robotino for robotino.\nPlease note that robotino is not yet supported by v3-generation of SmartSoft/SmartMDSD Toolchain."
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
# todo dennis
toolchain-update)
	echo -e "\n\n\n### Running toolchain update ...\n\n\n"
	sleep 2

	TC_DOWNLOAD=`tempfile`
	echo -e "\n# Downloading toolchain...\n"
	wget --progress=dot:mega --content-disposition $TOOLCHAIN_LATEST_URL -O $TC_DOWNLOAD || askabort

	echo -e "\n# Setting up toolchain...\n"

	mv ~/SOFTWARE/SmartMDSD_Toolchain.latest ~/SOFTWARE/SmartMDSD_Toolchain.`date +%Y-%m-%d` 
	mkdir -p ~/SOFTWARE/SmartMDSD_Toolchain.latest 
	cd ~/SOFTWARE/SmartMDSD_Toolchain.latest || askabort

	#TCNAME=$(ls * | sed "s/\.tar.*//g" | sed "s/\.bz2.*//g")
	tar xf $TC_DOWNLOAD || askabort
	EXECUTABLE=`pwd`/$(find ./ -name eclipse -type f)
	echo "Toolchain executable is: $EXECUTABLE"
	
	echo -e "\n# Setting up workspace path ...\n"
	IDEPREFS=$(find ./ -name org.eclipse.ui.ide.prefs)
	echo "MAX_RECENT_WORKSPACES=5
RECENT_WORKSPACES=$HOME/workspaces/SmartMDSD-Toolchain
RECENT_WORKSPACES_PROTOCOL=3
SHOW_WORKSPACE_SELECTION_DIALOG=true
eclipse.preferences.version=1
" > $IDEPREFS

	echo -e "\n# Creating desktop starter ...\n"
echo "[Desktop Entry]
Encoding=UTF-8
Version=1.0
Name=SmartMDSD Toolchain
Comment=Starts the latest version of the SmartMDSD Toolchain
Type=Application
Exec=$EXECUTABLE
Icon=
" > ~/Desktop/SmartMDSDToolchain.desktop
	chmod +x ~/Desktop/SmartMDSDToolchain.desktop
	exit 0
;;


###############################################################################
# Update the installation script
###############################################################################
script-update)
	echo -e "\n\n\n### Updating the script before starting it ...\n\n\n"
	T=`tempfile`
	echo "Tempfile: $T"
	
	wget "$SCRIPT_UPDATE_URL" -O $T
	cp $0 $T #################### FIXME!!!!!!!!!!!!!!!!

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
	if zenity --question --text="This installation/update script has an updater included.\nDo you want to update this script before installing it?\n\nWill update script from:\n$SCRIPT_UPDATE_URL\n"; then
		bash $SCRIPT_NAME script-update
	else
		echo -e "\n\n\n# Not updating the script before running it.\n\n\n"
	fi

	bash $SCRIPT_NAME start
;;


esac


