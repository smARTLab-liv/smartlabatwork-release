[ -z "$PS1" ] && return

# Basic options
# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

#export HISTCONTROL=ignoredups
export COLORFGBG='default;default'

declare -x CLICOLOR=1
declare -x LSCOLORS="DxGxFxdxCxdxdxhbadExEx"

# Aliases
alias ..='cd ..'
alias cd..='cd ..'
alias ...='cd ../..'
alias back='cd $OLDPWD'
alias dfh='df -h'
alias 'll'='ls -la'

# Use custom aliases
if [ -f ~/.bash_aliases ]; then
. ~/.bash_aliases
fi

# Prompt
BGREEN='\[\033[1;32m\]'
GREEN='\[\033[0;32m\]'
BRED='\[\033[1;31m\]'
RED='\[\033[0;31m\]'
BBLUE='\[\033[1;34m\]'
BLUE='\[\033[0;34m\]'
NORMAL='\[\033[00m\]'
BYELLOW='\[\033[1;33m\]'
PS1="${BYELLOW}\u${NORMAL}@${BRED}\h${NORMAL}:${BGREEN}\w${NORMAL} "

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    eval "`dircolors -b`"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

if [ -f /etc/bash_completion ]; then
    source /etc/bash_completion
fi


# make bash autocomplete with up arrow
bind '"\e[A":history-search-backward'
bind '"\e[B":history-search-forward'

#------------------------------------------------------------------------------
# ROS
#------------------------------------------------------------------------------
if [ -x /opt/ros/hydro ]; then
    source /opt/ros/hydro/setup.bash
    #export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH

    source ~/ros/devel/setup.bash
fi

export ROS_WORKSPACE=~/ros/src
   
export EDITOR='emacs'

alias 'rviz'='rosrun rviz rviz'
alias 'dynreconf'='rosrun rqt_reconfigure rqt_reconfigure'
alias 'emacs'='emacs -nw'
alias  sudo='sudo '
alias catkin_ws_make='catkin_make -C "$HOME/ros"'

function ros {
    export ROS_MASTER_URI=http://$1:11311
}

function rospath {
    export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH
}

function roswspath {
    export ROS_WORKSPACE=$1
}

ros slaw