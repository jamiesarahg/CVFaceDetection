import ridgeRegression
from sklearn.cross_validation import train_test_split
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt



def title(y_pred, y_test, target_names, i):
  "maps face numbers to target names"
  pred_name = target_names[y_pred[i]].rsplit(' ', 1)[-1]
  true_name = target_names[y_test[i]].rsplit(' ', 1)[-1]
  return 'predicted: %s\ntrue:      %s' % (pred_name, true_name)

def plot_gallery(images, titles, h, w, n_row=3, n_col=4):
    """Helper function for testing facial recognition to plot a gallery of portraits"""
    plt.figure(figsize=(1.8 * n_col, 2.4 * n_row))
    plt.subplots_adjust(bottom=0, left=.01, right=.99, top=.90, hspace=.35)
    for i in range(n_row * n_col):
        plt.subplot(n_row, n_col, i + 1)
        plt.imshow(images[i].reshape((h, w)), cmap=plt.cm.gray)
        plt.title(titles[i], size=12)
        plt.xticks(())
        plt.yticks(())

def test_facial_recognition_model(X_train, Y_faces):
  "runs facial recognition model with a training and test set and plots results"
  # splits data into training and testing set
  X_train, X_test, Y_train, y_test = train_test_split(X_train, Y_faces, test_size=0.25)
  # run the model that we established on the training set
  clf, pca = ridgeRegression.get_face_recognition_model(X_train, Y_train)

  #run pca on test data
  X_test_pca = pca.transform(X_test)
  y_pred = clf.predict(X_test_pca)


  with open('target_names.txt','r') as inf:
    target_names = eval(inf.read())
    n_classes = len(target_names)

  print(classification_report(y_test, y_pred, target_names=target_names))
  print(confusion_matrix(y_test, y_pred, labels=range(n_classes)))

  prediction_titles = [title(y_pred, y_test, target_names, i) for i in range(y_pred.shape[0])]

  h = 576
  w = 1
  plot_gallery(X_test, prediction_titles, h, w)

  # plot the gallery of the most significative eigenfaces

  # eigenface_titles = ["eigenface %d" % i for i in range(eigenfaces.shape[0])]

  # plot_gallery(eigenfaces, eigenface_titles, h, w)

  plt.show()
  

X_train, Y_train_direction, Y_train_name = ridgeRegression.get_training_data()
test_facial_recognition_model(X_train, Y_train_name)
