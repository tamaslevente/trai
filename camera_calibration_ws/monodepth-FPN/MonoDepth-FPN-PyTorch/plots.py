from cProfile import label
import numpy as np
import matplotlib.pyplot as plt


def create_plots(data, plot_name, data_info):
    save_dir="data_plots/"

    orig_data = data[:,0]
    pred_data = data[:,1]

    orig_mean = np.mean(orig_data)
    pred_mean = np.mean(pred_data)
    orig_mean_line = np.ones(orig_data.shape) * orig_mean
    pred_mean_line = np.ones(pred_data.shape) * pred_mean

    plt.plot(orig_data,'b', label="Original")
    plt.plot(pred_data,'r--', label="Predicted")
    plt.plot(orig_mean_line,'g-', label="Original mean")
    plt.plot(pred_mean_line,'m-',label="Predicted mean")
    plt.legend()

    plt.savefig(save_dir+plot_name)
    plt.show()
    plt.close()


    plt.hist(orig_data)
    plt.title("Original Data")
    plt.savefig(save_dir+data_info+" orginal data histogram")
    plt.show()
    plt.close()


    plt.hist(pred_data)
    plt.title("Predicted Data")
    plt.savefig(save_dir+data_info+" prediction data histogram")
    plt.show()
    plt.close()

    print("Standard deviation for " + data_info +" original data: ",np.std(orig_data))
    print("Standard deviation for " + data_info +" predicted data: ",np.std(pred_data))


if __name__=="__main__":

    test_data = np.loadtxt("curv_grad_orig_pred_test.csv", delimiter=",")
    create_plots(test_data, plot_name="curv_grad_test_data.png",data_info="Test")

    train_data = np.loadtxt("curv_grad_orig_pred_train.csv", delimiter=",")
    create_plots(train_data, plot_name="curv_grad_train_data.png", data_info="Train")

# plt.plot(epochs, val_loss_arr, 'b', label='Validation loss')
# plt.title('Training and Validation loss')
# plt.xlabel('Epochs')
# plt.ylabel('Loss')
# plt.legend()
# plt.savefig(save_dir+"t95_losses.png")
# plt.close()