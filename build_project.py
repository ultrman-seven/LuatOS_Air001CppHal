import os
import yaml
import argparse
import scripts.keilProj as kp

def createDir(dir):
    create_flag = True
    if os.path.exists(dir):
        if os.path.isdir(dir):
            create_flag = False
    if create_flag:
        os.mkdir(dir)


parse = argparse.ArgumentParser()
parse.add_argument('--cfgFile',type=str,default='./projCfg.yaml')
parse.add_argument('--dir',type=str,default='./projects')
args = parse.parse_args()
rf = open(file=args.cfgFile, mode='r', encoding='utf-8')
crf = rf.read()
rf.close()
yaml_data = yaml.load(stream=crf, Loader=yaml.FullLoader)
print(yaml_data)
