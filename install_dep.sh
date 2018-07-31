import yaml
import pdb
from urllib2 import urlopen
import os

ubuntu_version = 'xenial'

os.system('sudo apt-get install wget')
os.system('wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml')
os.system('wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml')
print "Opening base.yaml"
f = open('base.yaml')
print "Done!"
skip_packages = ['console_bridge', 'fastcdr fastrtps', 'libopensplice67', 'rti-connext-dds-5.3.1', 'urdfdom_headers']
f_shell = open('base.sh','wb')
x = yaml.load(f)
apt_context = 'sudo apt-get install -y --no-install-recommends '
for key, value in x.items():
    if key in skip_packages:
        continue
    for key2, value2 in value.items():
        if key2 != 'ubuntu':
            continue
        if isinstance(value2, dict):
            try:
                apt_context = apt_context + ' ' + value2[ubuntu_version][0]
            except:
                pass
        elif len(value2) != 0 :
            apt_context = apt_context + " " + value2[0]
print "Opening python.yaml"            
f2 = open("python.yaml")
print "Done!"
x = yaml.load(f2)
pip_context = ''
for key, value in x.items():
    if key in skip_packages:
        continue
    try:
        for key2, value2 in value.items():
            if key2 != 'ubuntu':
                continue
            if isinstance(value2, dict):
                if value2.keys() == ['pip']:
                    if isinstance(value2['pip'], dict):                   
                        pip_context = pip_context + "sudo pip install " + value2['pip']['packages'][0]+' -i https://pypi.douban.com/simples\n'
                    else:
                        pip_context = pip_context + "sudo pip install " + value2['pip'][0]+' -i https://pypi.douban.com/simples\n'
                elif isinstance(value2, dict):
                    if 'xenial' in value2.keys():
                        try:
                            apt_context = apt_context + " " + value2[ubuntu_version][0]
                        except:
                            if 'pip' in value2[ubuntu_version].keys():
                                if 'packages' in value2[ubuntu_version]['pip'].keys():
                                    pip_context = pip_context + 'sudo pip install ' + value2[ubuntu_version]['pip']['packages'][0]+' -i https://pypi.douban.com/simples\n'
                                else:
                                    pip_context = pip_context + 'sudo pip install ' + value2[ubuntu_version]['pip'][0]+' -i https://pypi.douban.com/simples\n'
                              
                           
                elif len(value2) != 0 :
                    apt_context = apt_context + " " + value2[0] 
    except:
        pdb.set_trace()
                
                
f_shell.write(apt_context+"\n"+pip_context)
f_shell.write('rm python.yaml\n')
f_shell.write('rm base.yaml\n')
f_shell.write('cd ~/ros2_ws\n')
f_shell.write('rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"')
f_shell.close()          

