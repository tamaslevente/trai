import torch
import torch.nn as nn
from pytorch_lightning import Trainer

import logging
import sys
sys.path.append("./models")
sys.path.append("./utils")

from interface import *
from utils import *
from vae import VAE
from beta_vae import betaVAE
import re

def get_trailing_number(s):
    m = re.search(r'\d+$', s)
    return str(int(m.group())) if m else None

def main(args):
    """ main() driver function """

    # Parameters parsing
    if filepath_is_not_valid(args.config):
        logging.error("The path {} is not a file. Aborting..".format(args.config))
        exit()

    configuration, architecture, hyperparameters = parse_config_file(args.config, args.variation)
    dataset_info = prepare_dataset(configuration)
    if (dataset_info is None):
        exit()

    # Initialization
    model = None
    if (args.variation == "VAE"):
        model = VAE(architecture, hyperparameters, dataset_info)
    elif (args.variation == "B-VAE"):
        model = betaVAE(architecture, hyperparameters, dataset_info)

    # here you can change the gpus parameter into the amount of gpus you want the model to use
    if hyperparameters["gpus"]==1:
        trainer = Trainer(max_epochs = hyperparameters["epochs"], gpus=hyperparameters["gpus"], fast_dev_run=False)
    else:
        trainer = Trainer(max_epochs = hyperparameters["epochs"], gpus=hyperparameters["gpus"],strategy="dp", fast_dev_run=False)

    # Training and testing
    if args.mode=="train":
        trainer.fit(model)
        print("Model is saved in: "+trainer.logger.log_dir)
        print("Running test:")            
        result = trainer.test(model)
        
    if args.mode=="eval":
        if args.model_version>=0:
            print("Using previously trained model:")
            latest_subdir="lightning_logs/version_"+str(args.model_version)
            entries = os.listdir(latest_subdir+"/checkpoints/")
            checkpoint = torch.load(latest_subdir+"/checkpoints/"+entries[0])
            print(latest_subdir)
            model.load_state_dict(checkpoint['state_dict'])
            
        else:
            print("Using latest model:")
            result = []
            b='lightning_logs'
            for d in os.listdir(b):
                bd = os.path.join(b, d)
                if os.path.isdir(bd): result.append(bd)
            latest_subdir = max(result, key=os.path.getmtime)
            entries = os.listdir(latest_subdir+"/checkpoints/")
            checkpoint = torch.load(latest_subdir+"/checkpoints/"+entries[0])
            print(latest_subdir)
            model.load_state_dict(checkpoint['state_dict'])
            
        result = trainer.test(model)

    if args.mode=="visualize":
        if args.model_version>=0:
            print("Using previously trained model:")
            latest_subdir="lightning_logs/version_"+str(args.model_version)
            entries = os.listdir(latest_subdir+"/checkpoints/")
            checkpoint = torch.load(latest_subdir+"/checkpoints/"+entries[0])
            print(latest_subdir)
            model.load_state_dict(checkpoint['state_dict'])
            
        else:
            print("Using latest model:")
            result = []
            b='lightning_logs'
            for d in os.listdir(b):
                bd = os.path.join(b, d)
                if os.path.isdir(bd): result.append(bd)
            latest_subdir = max(result, key=os.path.getmtime)
            entries = os.listdir(latest_subdir+"/checkpoints/")
            checkpoint = torch.load(latest_subdir+"/checkpoints/"+entries[0])
            print(latest_subdir)
            model.load_state_dict(checkpoint['state_dict'])
        # Model needs to be transferred to the cpu as sample and reconstruct are custom methods
        model = model.cpu()
        model.sample_depth(2,get_trailing_number(latest_subdir))
        model.reconstruct_depth(1,get_trailing_number(latest_subdir))

if __name__ == "__main__":
    """ call main() function here """
    print()
    # configure the level of the logging and the format of the messages
    logging.basicConfig(level=logging.ERROR, format="%(levelname)s: %(message)s\n")
    # parse the command line input
    args = parse_cmd_args()
    # call the main() driver function
    main(args)
    print("\n")
