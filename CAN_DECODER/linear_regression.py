# Import necessary libraries
import joblib
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

def load_and_train_model(model_path: str, data_path: str):
    # Load your dataset as a pandas DataFrame
    # Replace 'your_data.csv' with the actual filename or provide your data in another format
    data = pd.read_csv(data_path)

    # Define your features (input variables) and the target variable (output variable)
    X = data[['Bus Current', 'Pack Current', 'Pack Voltage', 'Vehicle Velocity']]
    y = data['Distance Driven']

    # Split the data into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # Initialize the linear regression model
    model = LinearRegression()

    # Train the model on the training data
    model.fit(X_train, y_train)

    # Make predictions on the test data
    y_pred = model.predict(X_test)

    # Evaluate the model's performance
    mse = mean_squared_error(y_test, y_pred)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    print(f"Mean Squared Error: {mse}")
    print(f"Mean Absolute Error: {mae}")
    print(f"R-squared (R^2) Score: {r2}")

    # You can also print the coefficients of the linear regression model
    print("Coefficients:", model.coef_)
    print("Intercept:", model.intercept_)

    # Save the model to a file
    joblib.dump(model, model_path)


def train_new_model(data_path: str):
    # Load your dataset as a pandas DataFrame
    # Replace 'your_data.csv' with the actual filename or provide your data in another format
    data = pd.read_csv(data_path)

    # Define your features (input variables) and the target variable (output variable)
    x = data[['Bus Current', 'Pack Current', 'Pack Voltage', 'Vehicle Velocity']]
    y = data['Distance Driven']

    # Split the data into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)

    # Initialize the linear regression model
    model = LinearRegression()

    # Train the model on the training data
    model.fit(X_train, y_train)

    # Make predictions on the test data
    y_pred = model.predict(X_test)

    # Evaluate the model's performance
    mse = mean_squared_error(y_test, y_pred)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    print(f"Mean Squared Error: {mse}")
    print(f"Mean Absolute Error: {mae}")
    print(f"R-squared (R^2) Score: {r2}")

    # You can also print the coefficients of the linear regression model
    print("Coefficients:", model.coef_)
    print("Intercept:", model.intercept_)

    # Save the model to a file
    print("writing model to file")
    joblib.dump(model, 'linear_regression.pk1')


def model_predict(model):
    # Load the model from the saved file
    model = joblib.load(model)  # Use joblib or pickle to load the model

    # Prepare new data for prediction (replace with actual data)
    new_data = pd.DataFrame({
        'Bus Current': [20],
        'Pack Current': [20],
        'Pack Voltage': [143],
        'Vehicle Velocity': [74]
    })

    # Use the loaded model to make predictions
    predicted_distance = model.predict(new_data)

    print(f"Predicted Distance Driven: {predicted_distance[0]}")


if __name__ == "__main__":
    #train_new_model("vehicle_data_gunpoint_sträcka_Max1.csv")
    #load_and_train_model("linear_regression.pk1", "vehicle_data_gunpoint_sträcka_Max2.csv")
    model_predict("linear_regression.pk1")