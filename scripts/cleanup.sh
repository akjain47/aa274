#!/us/bin/sh

git config --global --unset user.name
git config --global --unset user.email

gh auth logout

rm -rf ~/autonomy_ws