d:
	py -3 -m robotpy deploy
	
pid:
	py -3 -m robotpy deploy --skip-tests

sync:
	py -3 -m robotpy sync

info: 
	py -m robotpy deploy-info

list:
	py -m robotpy list

sim:
	python -m robotpy sim

delete:
	py -m robotpy undeploy

commit:
	git commit -asm "Computer Broke Unfunny"
	
push:
	git push origin main