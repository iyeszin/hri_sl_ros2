import numpy as np
import pandas as pd
from tqdm import tqdm
import json
import torch
from torch import nn
from torch.utils.data import DataLoader

# definition of FEATURE_COLUMNS, RHAND_IDX, and LHAND_IDX
LPOSE = [13,15,17,19,21]
RPOSE = [14,16,18,20,22]
POSE = LPOSE + RPOSE

X = [f'x_right_hand_{i}' for i in range(21)] + [f'x_left_hand_{i}' for i in range(21)] + [f'x_pose_{i}' for i in POSE]
Y = [f'y_right_hand_{i}' for i in range(21)] + [f'y_left_hand_{i}' for i in range(21)] + [f'y_pose_{i}' for i in POSE]
Z = [f'z_right_hand_{i}' for i in range(21)] + [f'z_left_hand_{i}' for i in range(21)] + [f'z_pose_{i}' for i in POSE]
FEATURE_COLUMNS = X+Y+Z

X_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if "x_" in col]
Y_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if "y_" in col]
Z_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if "z_" in col]

RHAND_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if "right" in col]
LHAND_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if  "left" in col]
RPOSE_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if  "pose" in col and int(col[-2:]) in RPOSE]
LPOSE_IDX = [i for i, col in enumerate(FEATURE_COLUMNS)  if  "pose" in col and int(col[-2:]) in LPOSE]


with open ("src/ros_sign_language_recognition/ros_sign_language_recognition/character_to_prediction_index.json", "r") as f:
    char_to_num = json.load(f)

# Add pad_token, start pointer and end pointer to the dict
pad_token = 'P'
start_token = '<'
end_token = '>'
pad_token_idx = 59
start_token_idx = 60
end_token_idx = 61

char_to_num[pad_token] = pad_token_idx
char_to_num[start_token] = start_token_idx
char_to_num[end_token] = end_token_idx
num_to_char = {j:i for i,j in char_to_num.items()}

device = 'cuda' if torch.cuda.is_available() else 'cpu'




def resize_pad_array(arr, target_frames=128, num_features=78):
    current_frames = arr.shape[0]
    if current_frames > target_frames:
        # Truncate the array if it has more than target_frames
        return arr[:target_frames]
    elif current_frames < target_frames:
        # Pad the array with zeros if it has less than target_frames
        padding = np.zeros((target_frames - current_frames, num_features))
        return np.vstack([arr, padding])
    else:
        # Return the array as is if it already matches the target_frames
        return arr

def dominant_hand(row):
    lnan = np.count_nonzero(np.isnan(row['left']))
    rnan = np.count_nonzero(np.isnan(row['right']))
    if lnan < rnan:
        # print('left')
        return row['left']
    else:
        # print('right')
        return row['right']
    

def create_sign_dataset(sign_data, padding_value=0.0, max_length=64):
    dataset = []
    for sign_sequence in sign_data:
        
        # Convert the sign sequence into a torch tensor
        sign_tensor = torch.tensor(sign_sequence, dtype=torch.float32)

        # Replace NaN values with zeros
        sign_tensor[torch.isnan(sign_tensor)] = padding_value
        
        # Pad the sequence if its length is less than max_length
        if sign_tensor.shape[0] < max_length:
            padding = torch.full((max_length - sign_tensor.shape[0], *sign_tensor.shape[1:]), padding_value)
            sign_tensor = torch.cat([sign_tensor, padding])
        
        # Append the sign data to the dataset
        dataset.append(sign_tensor)

    return dataset



def inference_collate_fn(data):
    sign_data = []
    
    for sign_vector in data:
        sign_data.append(torch.tensor(sign_vector))  # Ensure sign_vector is in the correct form
    
    # Stack sign_data to create a batched tensor
    sign_data = torch.stack(sign_data)
    sign_data = sign_data.transpose(0, 1)
    
    return sign_data

def process_data():
    file_id = "landmarks"  # Specify the file ID for inference
    file_df = pd.read_csv(f"./{file_id}.csv")  # Load CSV file
    data_to_collect = []

    # Assuming 'frame_id' is the column containing frame IDs in your CSV file

    for frame_id in file_df['frame']:
        # Assuming the CSV file contains columns for hand landmarks and they are already filled with zeros

        # Prepare data entry for each sequence
        data_entry = {}

        # Flatten and store each feature's values
        for column_name in FEATURE_COLUMNS:
            data_entry[column_name] = file_df[column_name].values.flatten()

        # Append this entry to our collection list
        data_to_collect.append(data_entry)

    # Convert collected data into a DataFrame
    hands_df = pd.DataFrame(data_to_collect)

    # Select columns for 'right' including specified pose columns
    right_hand_cols = [col for col in hands_df.columns if 'right_hand' in col]
    right_pose_cols = [f'{dim}_pose_{i}' for i in [14, 16, 18, 20, 22] for dim in ['x', 'y', 'z']]
    right_cols = right_hand_cols + right_pose_cols

    # Select columns for 'left' including specified pose columns
    left_hand_cols = [col for col in hands_df.columns if 'left_hand' in col]
    left_pose_cols = [f'{dim}_pose_{i}' for i in [13, 15, 17, 19, 21] for dim in ['x', 'y', 'z']]
    left_cols = left_hand_cols + left_pose_cols

    # Function to concatenate values of given columns for a row
    def concat_cols(row, cols):
        return [row[col] for col in cols]

    # Concatenate values for 'right' and 'left'
    hands_df['right'] = hands_df.apply(concat_cols, cols=right_cols, axis=1)
    hands_df['left'] = hands_df.apply(concat_cols, cols=left_cols, axis=1)

    # Create a new DataFrame with desired columns
    hands_df = hands_df[[ 'right', 'left']].copy()

    hands_df['right'] = hands_df['right'].apply(lambda x: np.array(x).T)
    hands_df['left'] = hands_df['left'].apply(lambda x: np.array(x).T)

    # Apply the normalization function to each row for 'right' and 'left' columns
    for index, row in tqdm(hands_df.iterrows()):
        hands_df.at[index, 'right'] = resize_pad_array(row['right'])
        hands_df.at[index, 'left'] = resize_pad_array(row['left'])

    # # Verify the changes
    # for index, row in hands_df.iterrows():
    #     print(f"Right shape: {row['right'].shape}, Left shape: {row['left'].shape}")
    #     break

    frame_num = 128
    num_rows = len(hands_df)

    for row in tqdm(range(num_rows)):
        
        for frame in range(frame_num):
            left_row_frame = hands_df.left[row][frame]
            # [:21 x_hand,21:42 y_hand,42:63 z_hand,63:68 x_pose, 68:73 y_pose 73:78 z_pose]
            left_row_frame[:21] = 1 -  left_row_frame[:21]
            left_row_frame[63:68] = 1 - left_row_frame[63:68] 
            hands_df.left[row][frame] = left_row_frame

    hands_df['sign'] = hands_df.apply(dominant_hand, axis=1)

    dataset = create_sign_dataset(hands_df['sign'])

    return dataset



# MODEL
class LandmarkEmbedding(nn.Module):
    def __init__(self, num_hid=64, maxlen=100):
        super(LandmarkEmbedding, self).__init__()
        # Define the convolutional layers
        self.conv1 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        self.conv2 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        self.conv3 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        
        # Define the positional embedding layer
        self.pos_emb = nn.Embedding(num_embeddings=maxlen, embedding_dim=num_hid)
        
        # Activation function
        self.relu = nn.ReLU()

    def forward(self, x):
        # Apply the convolutional layers with ReLU activation
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        
        return x

import json
import random
import torch
from torch import nn

with open ("src/ros_sign_language_recognition/ros_sign_language_recognition/character_to_prediction_index.json", "r") as f:
    char_to_num = json.load(f)

# Add pad_token, start pointer and end pointer to the dict
pad_token = 'P'
start_token = '<'
end_token = '>'
pad_token_idx = 59
start_token_idx = 60
end_token_idx = 61

char_to_num[pad_token] = pad_token_idx
char_to_num[start_token] = start_token_idx
char_to_num[end_token] = end_token_idx
num_to_char = {j:i for i,j in char_to_num.items()}

print(char_to_num)


class LandmarkEmbedding(nn.Module):
    def __init__(self, num_hid=64, maxlen=100):
        super(LandmarkEmbedding, self).__init__()
        # Define the convolutional layers
        self.conv1 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        self.conv2 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        self.conv3 = nn.Conv1d(in_channels=num_hid, out_channels=num_hid, kernel_size=11, stride=2, padding=5)
        
        # Define the positional embedding layer
        self.pos_emb = nn.Embedding(num_embeddings=maxlen, embedding_dim=num_hid)
        
        # Activation function
        self.relu = nn.ReLU()

    def forward(self, x):
        # Apply the convolutional layers with ReLU activation
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        
        return x

class Encoder(nn.Module):
    def __init__(self,input_size,hidden_size,num_layers,dropout_ratio):
        super(Encoder,self).__init__()
        # intialization of hyperparameters
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.dropout_ratio = dropout_ratio
        
        # intialization of layers
        self.dropout_layer = nn.Dropout(dropout_ratio)
        self.embedding_layer = LandmarkEmbedding()
        self.rnn = nn.LSTM(input_size=10,hidden_size=hidden_size,num_layers=num_layers,dropout=dropout_ratio)
    
    def forward(self,x:torch.Tensor)->torch.Tensor :
        embedding = self.dropout_layer(self.embedding_layer(x))
        out,(hidden,cell) = self.rnn(embedding)
        
        return hidden,cell

            
class Decoder(nn.Module):
    def __init__(self, input_size, embedding_size, hidden_size, num_layers, output_size, dropout_ratio):
        super(Decoder, self).__init__()
        self.input_size = input_size
        self.embedding_size = embedding_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.output_size = output_size
        self.dropout_ratio = dropout_ratio

        self.dropout_layer = nn.Dropout(dropout_ratio)
        self.embedding_layer = nn.Embedding(num_embeddings=input_size, embedding_dim=embedding_size)
        self.rnn = nn.LSTM(input_size=embedding_size, hidden_size=hidden_size, num_layers=num_layers, dropout=dropout_ratio)
        self.fc = nn.Linear(in_features=hidden_size, out_features=output_size)

    def forward(self, x: torch.Tensor, hidden, cell) -> torch.Tensor:
        x = x.unsqueeze(0)
        embedding = self.dropout_layer(self.embedding_layer(x))
        out, (hidden, cell) = self.rnn(embedding, (hidden, cell))
        predictions = self.fc(out)
        predictions = predictions.squeeze(0)

        return predictions, hidden, cell

    def inference(self, start_token, hidden, cell, max_length=100):
        """Generate output token by token during inference."""
        # Initialize the input token with the start token
        input_token = torch.tensor(start_token).unsqueeze(0)  # Assuming start_token is an integer
        outputs = []

        for _ in range(max_length):
            # Forward pass through the decoder
            output, hidden, cell = self.forward(input_token, hidden, cell)
            # Get the predicted token (argmax)
            predicted_token = output.argmax(dim=1)
            # Append the predicted token to the outputs list
            outputs.append(predicted_token.item())

            # Prepare the input token for the next time step
            input_token = predicted_token.unsqueeze(0)

        return outputs


class Seq2Seq(nn.Module):
    def __init__(self, encoder, decoder):
        super(Seq2Seq, self).__init__()
        self.encoder = encoder
        self.decoder = decoder
        
    def forward(self, source, target=None, teacher_force_ratio=0.5):
        if target is not None:
            # Training mode, perform teacher forcing
            batch_size = source.shape[1]
            target_len = target.shape[0]
            target_vocab_size = len(char_to_num)
            
            outputs = torch.zeros(target_len, batch_size, target_vocab_size)
            
            hidden, cell = self.encoder(source)
            
            x = target[0]
            for t in range(1, target_len):
                output, hidden, cell = self.decoder(x, hidden, cell)
                
                outputs[t] = output
                best_guess = output.argmax(1)
                
                x = target[t] if random.random() < teacher_force_ratio else best_guess
            
            return outputs
        else:
            print("inference")
            # Inference mode, generate output based on source only
            MAX_TARGET_LEN = 10
            batch_size = source.shape[1]
            target_len = MAX_TARGET_LEN  # Define maximum target length for inference
            target_vocab_size = len(char_to_num)
            outputs = torch.zeros(target_len, batch_size, target_vocab_size)
            
            hidden, cell = self.encoder(source)
            x = torch.zeros(batch_size, dtype=torch.long).fill_(start_token_idx).to(source.device)  # Start token
            for t in range(1, target_len):
                output, hidden, cell = self.decoder(x, hidden, cell)
                outputs[t] = output
                x = output.argmax(1)  # Use predicted token as input for the next time step
                
                # Check for stopping condition (e.g., generating end token)
                if (x == end_token_idx).all():
                    break
            
            return outputs



def inf_model():

    num_hid=64
    encoder_input_size = num_hid
    encoder_hidden_size = 1024
    encoder_num_layers = 2
    encoder_dropout_ratio = 0.5

    encoder = Encoder(input_size=encoder_input_size,
                    hidden_size=encoder_hidden_size,
                    num_layers=encoder_num_layers,
                    dropout_ratio=encoder_dropout_ratio).to(device)

    decoder_input_size = len(char_to_num)
    decoder_embedding_size = len(char_to_num)
    decoder_hidden_size = 1024
    decoder_num_layers = 2
    decoder_output_size = len(char_to_num)
    decoder_dropout_ratio = 0.5
    decoder = Decoder(input_size=decoder_input_size,
                    embedding_size = decoder_embedding_size,
                    hidden_size=decoder_hidden_size,
                    num_layers=decoder_num_layers,
                    output_size=decoder_output_size,
                    dropout_ratio=decoder_dropout_ratio).to(device)


    best_model = Seq2Seq(encoder,decoder)
    # print(best_model)

    try:
        check_point = torch.load('src/ros_sign_language_recognition/ros_sign_language_recognition/weight/best_Seq2Seq_model.pth')
        best_model.load_state_dict(check_point)
    except FileNotFoundError:
        print("Error: Model checkpoint file not found.")
    except Exception as e:
        print("Error loading model checkpoint:", e)

    # check_point = torch.load('src/ros_sign_language_recognition/ros_sign_language_recognition/weight/best_Seq2Seq_model.pth')
    # check_point.keys()
    # best_model.load_state_dict(check_point)

    return best_model



def main():
    dataset = process_data()
    
    inf_dataloader = DataLoader(dataset=dataset, batch_size=64, collate_fn=inference_collate_fn, drop_last=True)
    best_model = inf_model()

    all_output_sentences = []

    with torch.no_grad():
        for batch in inf_dataloader:
            batch = batch.to(device)
            output = best_model(batch)
            output = output.permute(1, 0, 2)  # Reshape tensor
            output = output.argmax(2)  # Get indices with highest probability
            num_to_char = {j: i for i, j in char_to_num.items()}
            
            # Process only the first sequence
            output_text = ''.join([num_to_char[_] for _ in output[0].cpu().detach().numpy() if _ != end_token_idx])
            all_output_sentences.append(output_text)

    # # Print the last output sentence
    # print(all_output_sentences[-1])

    return all_output_sentences[-1]


    
if __name__ == '__main__':
    main()
