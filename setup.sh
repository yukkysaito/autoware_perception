echo -e ">  Do you run setup? This will take a while. (y/n)? "
read answer
sudo apt install cowsay -y
if echo "$answer" | grep -iq "^y"
then
  SCRIPT_DIR=$(cd $(dirname $0); pwd)
  cd $SCRIPT_DIR/ansible
  sudo apt install ansible
	ansible-playbook -i localhost, $SCRIPT_DIR/ansible/localhost-setup-devpc.yml
	echo -e "\e[32mok: Complete \e[0m"
else
    echo -e "\e[33merror: Cannot Complete \e[0m"
fi