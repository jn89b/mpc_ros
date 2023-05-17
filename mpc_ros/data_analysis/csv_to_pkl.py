import pickle as pkl
import pandas as pd

# Read csv file
df = pd.read_csv('log/shes_the_one.csv')

# Convert to pkl file
df.to_pickle('shes_the_one.pkl')
