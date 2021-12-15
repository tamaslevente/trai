import __init__
from tqdm import tqdm
import numpy as np
import torch
import torch_geometric.datasets as GeoData
from torch_geometric.data import DenseDataLoader
import torch_geometric.transforms as T
from config import OptInit
from architecture import DenseDeepGCN
from utils.ckpt_util import load_pretrained_models
import logging
import open3d as o3d

color_map_255 = {
    0: np.array([100,204,42]),
    1: np.array([50,104,250]),
    2: np.array([105,244,244]),
    3: np.array([249,239,60]),
    4: np.array([227,109,223]),
    5: np.array([100,106,249]),
    6: np.array([199,200,98]),
    7: np.array([159,113,191]),
    8: np.array([224,62,31]),
    9:np.array([201,95,99]),
    10:np.array([95,202,103]),
    11:np.array([200,200,200]),
    12:np.array([50,50,50]),
}

color_map = {key : color_map_255[key]/255 for key in color_map_255}


def main():
    opt = OptInit().get_args()

    logging.info('===> Creating dataloader...')
    test_dataset = GeoData.S3DIS(opt.data_dir, opt.area, train=False, pre_transform=T.NormalizeScale())
    test_loader = DenseDataLoader(test_dataset, batch_size=opt.batch_size, shuffle=False, num_workers=0)
    opt.n_classes = test_loader.dataset.num_classes
    if opt.no_clutter:
        opt.n_classes -= 1

    logging.info('===> Loading the network ...')
    model = DenseDeepGCN(opt).to(opt.device)
    model, opt.best_value, opt.epoch = load_pretrained_models(model, opt.pretrained_model, opt.phase)

    logging.info('===> Start Evaluation ...')
    test(model, test_loader, opt)
    


def save_pcd(pos, labels, name, count):
    '''
    Issues due to crude implementation, as this is a test run:
    The issue is that I don't take into account the batches, hence why I use "labels[0]", to work with the first
    image from the batch. This is improper and it will also crash in the case that the batch size is 1.
    This can be easily altered to take into account the batches (just segment the labels given the batch size), but
    it's not necessary at this very moment.
    '''
    global color_map
    labels = np.reshape(labels[0], (4096, 1))
    colors = np.array([color_map[label[0]] for label in labels])
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pos)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud("./data/personal/" + name + "_" + str(count) + ".pcd", pcd)
    


def test(model, loader, opt):
    Is = np.empty((len(loader), opt.n_classes))
    Us = np.empty((len(loader), opt.n_classes))

    model.eval()
    data_iter = iter(loader) 

    with torch.no_grad():
        for i in range(10):
            data = next(data_iter)
            inputs = torch.cat((data.pos.transpose(2, 1).unsqueeze(3), data.x.transpose(2, 1).unsqueeze(3)), 1)
            gt = data.y

            out = model(inputs)
            pred = out.max(dim=1)[1]

            pred_np = pred.cpu().numpy()
            target_np = gt.cpu().numpy()
            
            save_pcd(data.cpu().pos[0].numpy(), target_np, "target", i);
            save_pcd(data.cpu().pos[0].numpy(), pred_np, "pred", i);

            for cl in range(opt.n_classes):
                cur_gt_mask = (target_np == cl)
                cur_pred_mask = (pred_np == cl)
                I = np.sum(np.logical_and(cur_pred_mask, cur_gt_mask), dtype=np.float32)
                U = np.sum(np.logical_or(cur_pred_mask, cur_gt_mask), dtype=np.float32)
                Is[i, cl] = I
                Us[i, cl] = U

    ious = np.divide(np.sum(Is, 0), np.sum(Us, 0))
    ious[np.isnan(ious)] = 1
    for cl in range(opt.n_classes):
        logging.info("===> mIOU for class {}: {}".format(cl, ious[cl]))
    logging.info("===> mIOU is {}".format(np.mean(ious)))


if __name__ == '__main__':
    main()


