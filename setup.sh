echo -e ">  Do you run setup? This will take a while. (y/n)? "
read answer
sudo apt install cowsay -y
if echo "$answer" | grep -iq "^y"
then
  cd ./ansible
  sudo apt install ansible
	ansible-playbook -i localhost, ./localhost-setup-devpc.yml
	echo -e "\e[32mok: Complete \e[0m"
else
    echo -e "\e[33merror: Cannot Complete \e[0m"
fi