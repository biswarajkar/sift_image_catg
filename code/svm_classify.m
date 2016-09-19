% Starter code referred from code by James Hays and Sam Birch from Brown University

%This function will train a linear SVM for every category (i.e. one vs all)
%and then use the learned linear classifiers to predict the category of
%every test image. Every test feature will be evaluated with all 15 SVMs
%and the most confident SVM will "win".

function predicted_categories = svm_classify(training_image_features, training_labels, test_image_features)
% image_feats is an N x d matrix, where d is the dimensionality of the
%  feature representation.
% train_labels is an N x 1 cell array, where each entry is a string
%  indicating the ground truth category for each training image.
% test_image_feats is an M x d matrix, where d is the dimensionality of the
%  feature representation.
% predicted_categories is an M x 1 cell array, where each entry is a string
%  indicating the predicted category for each test image.

LAMBDA = 0.00001;
fprintf('      LAMBDA for SVN: %.5f\n\n', LAMBDA);

labels = unique(training_labels);
num_of_categories = length(labels);
training_score = [];

for catNum = 1:num_of_categories
    %{
    matching_indices = strcmp(string, cell_array_of_strings)
    This function which indices in train_labels match a particular category.
    This is useful for creating the binary labels for each SVM training task.
    %}
    matching_indices = strcmp(labels(catNum), training_labels);
    matching_indices = +matching_indices;
    matching_indices(matching_indices==0) = -1;
    %{
    [W B] = vl_svmtrain(features, labels, LAMBDA)
    http://www.vlfeat.org/matlab/vl_svmtrain.html
    This function trains linear svms based on training examples, binary
    labels (-1 or 1), and LAMBDA which regularizes the linear classifier
    by encouraging W to be of small magnitude.
    %}
    [W, B] = vl_svmtrain(training_image_features', matching_indices, LAMBDA);
    
    %Confidence, or distance from the margin, is W*X + B where '*' is the
    %inner product or dot product and W and B are the learned hyperplane parameters.
    training_score = [training_score; W'*test_image_features' + B];
end

%[Y,I] = MAX(X) returns the indices of the maximum values in vector I.
[~,label_indices] = max(training_score);

%Get the name of the most matched category for each image
predicted_categories = labels(label_indices');

end


