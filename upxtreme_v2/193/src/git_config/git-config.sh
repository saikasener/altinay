#!/bin/bash

# Git Configuration [BEGIN]

# Save the credentials for an hour
git config --global credential.helper 'cache --timeout=3600'

# Enable tab completion
source ~/git-completion.bash

# colors!
green="\[\033[0;32m\]" # 0 - normal 
blue="\[\033[1;96m\]"  # 1 - bold
purple="\[\033[0;35m\]"
reset="\[\033[0m\]"
yellow="\[\033[1;93m\]"

# Change command prompt
source ~/git-prompt.sh 
export GIT_PS1_SHOWDIRTYSTATE=1
# '\u' adds the name of the current user to the prompt
# '\$(__git_ps1)' adds git-related stuff
# '\W' adds the name of the current directory
export PS1="$yellow\u$green\$(__git_ps1)$blue \W $ $reset"

# Git Configuration [END]

