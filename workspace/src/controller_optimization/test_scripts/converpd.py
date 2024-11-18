import pandas as pd


df = pd.read_csv('output_result_final.csv')

print(df.head())
print(type(df['id'][10]))


def convert(x):
    if '(' in x:
        s = x.split(',')[0].strip('() ')
        return s


df['id'] = df['id'].apply(lambda x: convert(x))
df['id'][0] = 'reference'

df.to_csv('output_result_final_converted.csv', index=False, header=True)
print(df.head())
