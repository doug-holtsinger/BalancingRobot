# BalancingRobot
Balancing Robot

Commands to clone repository:

git clone git@github.com:doug-holtsinger/BalancingRobot.git --recurse-submodules
git submodule update --init
git submodule foreach -q --recursive 'git checkout $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master)'
git config --global status.submoduleSummary true
