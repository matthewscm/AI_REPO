# Read data from CSV and convert to feature vectors
import csv

# Create a dataset list to hold feature vectors and labels
dataset = []

# Open the CSV file and read its contents
with open("data/increased_recycling_dataset.csv", "r") as file: # Change "data.csv" to updated file name
    # Use csv.reader to read the file
    reader = csv.reader(file)
    # Skip the header row
    header = next(reader)  # skip header as that is for our reference only
    # Iterate through each row in the CSV
    for row in reader:
        # Check if the row has enough columns
        if len(row) < 7:
            continue

        # IGNORE "Product" (row[0])
        features = [
            float(row[1]),  # Height
            float(row[2]),  # Width
            float(row[3]),  # H
            float(row[4]),  # S
            float(row[5])   # V
        ]
        # Extract the label (Recyclable Flag)
        label = row[6]  # Recyclable Flag
        # Append the features and label as a tuple to the dataset
        dataset.append((features, label))

# Print for verification
for features, label in dataset:
    print(f"Features: {features} | Label: {label}")

# ------------------------------------------------
# SVM Model Training and Evaluation (Below) Maddy's Code
#-------------------------------------------------
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# Step 1:Collect Data for Training and Testing (From Claire) (Above)
# Separate features and labels
X = [features for features, label in dataset]
y = [label for features, label in dataset]


# --- Step 2: Split the dataset into training and testing sets with Stratification---
# test_size=0.3 gives more samples to the test set (4-5 samples instead of 3)
# stratify=y ensures both classes exist in the test set
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.3, random_state=42, stratify=y
)

# --- Step 3: Feature Scaling (Required for RBF Kernels) ---
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train) # Scale features so that they are all considered equally by the SVM model (important for RBF kernel)
X_test = scaler.transform(X_test) # Scale features so that they are all considered equally by the SVM model (important for RBF kernel)

print(f"Training set size: {len(X_train)}")
print(f"Testing set size: {len(X_test)}")

# --- Step 4 & 5: SVM Implementation and Training ---
# Step 4: Create an SVM classifier choosing a kernel function
from sklearn import svm
classifier = svm.SVC(kernel='rbf', gamma='scale', C=10.0) # SVM with RBF kernel (often performs better than linear for complex data)
# Step 5: Train the SVM classifier on the training data
classifier.fit(X_train, y_train)

# Step 6: Predict the labels "Recyclable" or "Not Recyclable" for the testing set
y_pred = classifier.predict(X_test)

# Step 7: Evaluate the model's performance using metrics such as accuracy, precision, recall, and F1-score
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
# --- Step 7: Evaluate ---
print("\nClassification Report:")
# zero_division=0 prevents the warnings you saw earlier
print(classification_report(y_test, y_pred, zero_division=0))

print("Confusion Matrix:")
print(confusion_matrix(y_test, y_pred))

print(f"\nAccuracy Score: {accuracy_score(y_test, y_pred):.2%}")

# Step 8: Feed new data points into the trained model to predict their recyclability
# Example new data points/features from (RUBY) (Height, Width, H, S, V)
new_dataset = []

# Open the fake test CSV file
with open("data/test_sorting_data.csv", "r") as test_file:
    test_reader = csv.reader(test_file)
    # Skip header
    test_header = next(test_reader)
    
    for row in test_reader:
        # Based on the CSV I provided: [F1, F2, F3, F4, F5, Label]
        # We take the first 5 columns as features
        features = [float(val) for val in row[:5]]
        # The 6th column (index 5) is the label
        true_label = row[5]
        new_dataset.append((features, true_label))

# Separate features for prediction
X_new = [f for f, l in new_dataset]
y_true = [l for f, l in new_dataset]

# --- IMPORTANT: Apply the same scaling used in training ---
X_new_scaled = scaler.transform(X_new)

# Run predictions
new_predictions = classifier.predict(X_new_scaled)

# Print results in a clear format
print("\n" + "="*30)
print("NEW DATA TEST RESULTS")
print("="*30)
for i in range(len(new_dataset)):
    print(f"Features: {X_new[i]} | Predicted: {new_predictions[i]} | Actual: {y_true[i]}")





