#!/usr/bin/env python3
import pandas as pd
import numpy as np
from odf.opendocument import load, OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell, TableColumn
from odf.style import Style, TableColumnProperties
from odf.text import P
import math


def orientation_error(amcl_orientation, gt_orientation):
    # Values already in gradians
    error = np.abs(amcl_orientation - gt_orientation)
    return np.minimum(error, 2 * np.pi - error)


def error_position(ground_truth_x, ground_truth_y, amcl_x, amcl_y):
    return math.sqrt((ground_truth_x - amcl_x)**2 + (ground_truth_y - amcl_y)**2)


def read_ods(file_path, sheet_name):
    # Load the ODS file
    doc = load(file_path)
    data = []
    
    # Iterate over all sheets in the file
    for sheet in doc.spreadsheet.getElementsByType(Table):
        if sheet.getAttribute("name") == sheet_name:
            # Process each row
            for row in sheet.getElementsByType(TableRow):
                row_data = []
                for cell in row.getElementsByType(TableCell):
                    cell_value = ""
                    for p in cell.getElementsByType(P):
                        cell_value += str(p)
                    
                    # If the value is an empty string, handle it as a null value
                    if cell_value == "":
                        cell_value = None
                    
                    # Add the value to the row
                    row_data.append(cell_value)
                data.append(row_data)

    # Correct error of duplicate numbers
    if sheet_name == "L1x6_L1x4_mapped_V2":
        data[1].insert(0, '1')
        data[7].insert(0, '2')
        data[17].insert(0, '4')
        data[22].insert(0, '5')

    else:
        data[1].insert(0, '1')
        data[7].insert(0, '2')
        data[13].insert(0, '3')
        data[19].insert(0, '4')
        data[25].insert(0, '5')

    # If there is data, convert it to a DataFrame
    if data:
        df = pd.DataFrame(data[1:], columns=data[0])
        
        # Convert all possible columns to numbers, even if they are text
        df = df.apply(lambda x: pd.to_numeric(x, errors='ignore'))
        # df['Cycle'] = df['Cycle'].astype(int)
        if df.isnull().any().any():
            print(f"Value NaN in DataFrame {sheet_name}")
        
        return df
    else:
        return pd.DataFrame()




d=True
if d==True: #mur620d
    file_path = '/home/rafass/Documents/Bachelorarbeit/Posiciones/mur620d/positions.ods'
    objects=["NoObject",
            "Experiment8","Experiment8_mapped" ]


else:     #mur620

    file_path = '/home/rafass/Documents/Bachelorarbeit/Posiciones/mur620/positions_object.ods'
    objects=[ "0x0x0",
            "0.5x1x1", "0.5x1x1_map",
            "1x1x1", "1x1x1_map",
            "2x1x1", "2x1x1_map",
            "2x1.5x1", "2x1.5x1_map",
            "4x1x1", "4x1x1_map"]
    
number_decimals = 5

# Columns Table 1
columns_table = ["Object Name", "Average Pos Error (m)", "Min Pos Error (m)", "Max Pos Error (m)", "Point Max Error","SD Position (m)",
                 "Average Ori Error (°)", "Min Ori Error(°)", "Max Ori Error(°)",  "Point Max Ori Error", "SD Orientation (°)"]

table = Table(name="Accuracy")
header_row = TableRow()
for column in columns_table:
    cell = TableCell()
    cell.addElement(P(text=str(column)))
    header_row.addElement(cell)
table.addElement(header_row)


#  Columns Table 2
columns_table2 = ["Object Name"]
for x in range(1,6):
    columns_table2.append( f'{x}: Accuracy (m)')
    columns_table2.append( f'{x}: Repeatibility')
    columns_table2.append( f'{x}: Average Error (m)')
    columns_table2.append( f'{x}: Min Pos Error (m)')
    columns_table2.append( f'{x}: Max Pos Error (m)')
    columns_table2.append( f'{x}: SD (m)')
columns_table2.append('Worst Point')
table2 = Table(name="Position Stadistics")
header_row2 = TableRow()
for column in columns_table2:
    cell = TableCell()
    cell.addElement(P(text=str(column)))
    header_row2.addElement(cell)   
table2.addElement(header_row2)


#  Columns Table 3
columns_table3 = ["Object Name"]
for x in range(1,6):
    columns_table3.append( f'{x}: Average Error (°)')
    columns_table3.append( f'{x}: Min Error (°)')
    columns_table3.append( f'{x}: Max Error (°)')
    columns_table3.append( f'{x}: SD (°)')
columns_table3.append('Worst Point')
table3 = Table(name="Orientation Stadistics")
header_row3 = TableRow()
for column in columns_table3:
    cell = TableCell()
    cell.addElement(P(text=str(column)))
    header_row3.addElement(cell)   
table3.addElement(header_row3)

for dimension in objects:

    try:
        data = read_ods(file_path, dimension)
    except Exception as e:
        print(f'Error reading file: {e}')
        print(f'With object: {dimension}')
        exit()

    if not data.empty:
        num_rows = data.shape[0]
        if num_rows != 0:
            
            ############################################# Delete Configuration 4 for mur620d #########################################
           
            if dimension in ['NoObject','Experiment8', 'Experiment8_mapped']:
                # Delete rows where column configuration number is 4
                data = data[data['Configuration number'] != 4]

            ############################################# Table 1 #########################################

            # Calculate position error
            data['Position Error'] = data.apply(lambda row: error_position(row['Ground Truth x'],
                                                                           row['Ground Truth y'], row['Amcl Pose x'], row['Amcl Pose y']), axis=1)
            mean_error_position = data['Position Error'].mean()
            mean_error_position = round(mean_error_position, number_decimals)
            min_error_pos = data['Position Error'].min()
            min_error_pos = round(min_error_pos, number_decimals)
            # Point with biggest Position Error
            max_error_pos = data['Position Error'].max()
            max_error_pos_row = data[data['Position Error'] == max_error_pos]
            cycle_info = max_error_pos_row['Cycle'].values[0]
            config_info = max_error_pos_row['Configuration number'].values[0]
            point_max_error=f"Cycle: {cycle_info}, Configuration: {config_info}"
            max_error_pos = round(max_error_pos, number_decimals)

            std_deviation_pos = data['Position Error'].std()
            std_deviation_pos = round(std_deviation_pos, number_decimals)

            # Calculate orientation error
            data['Orientation Error'] = data.apply(lambda row: orientation_error(
                                        row['Amcl Pose Orientation'], row['Ground Truth Orientation']), axis=1)
            mean_error_orientation = data['Orientation Error'].mean()
            mean_error_orientation = math.degrees(mean_error_orientation)
            mean_error_orientation = round(mean_error_orientation, number_decimals)
            min_error_ori = data['Orientation Error'].min()
            min_error_ori = math.degrees(min_error_ori)
            min_error_ori = round(min_error_ori, number_decimals)
            max_error_ori = data['Orientation Error'].max()

            # Point with biggest Orientation Error
            max_error_ori_row = data[data['Orientation Error'] == max_error_ori]
            cycle_info = max_error_ori_row['Cycle'].values[0]
            config_info = max_error_ori_row['Configuration number'].values[0]
            point_max_error_ori=f"Cycle: {cycle_info}, Configuration: {config_info}"
            max_error_pos = round(max_error_pos, number_decimals)


            max_error_ori = math.degrees(max_error_ori)
            max_error_ori = round(max_error_ori, number_decimals)
            std_deviation_ori = data['Orientation Error'].std()
            std_deviation_ori=math.degrees(std_deviation_ori)
            std_deviation_ori = round(std_deviation_ori, number_decimals)


            row = TableRow()  # New Row

            cell = TableCell()
            cell.addElement(P(text=str(dimension)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(mean_error_position)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(min_error_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(max_error_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(point_max_error)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(std_deviation_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(mean_error_orientation)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(min_error_ori)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(max_error_ori)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(point_max_error_ori)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(std_deviation_ori)))
            row.addElement(cell)
            table.addElement(row)

            ############################################# Table 2 ###############################################
            x_amcl=[]
            y_amcl=[]
            mean_error_positions=[]
            min_error_positions=[]
            max_error_positions=[]
            row_table2 = TableRow()
            cell = TableCell()
            cell.addElement(P(text=str(dimension)))
            row_table2.addElement(cell)
            biggest_mean_error=0
            a=0     #Point with biggest mean error position
            for x in range (1,6):
                filtered_data = data[data['Configuration number'] == x]
                #Barycenter for AMCL and Ground Truth
                mean_x_amcl=filtered_data['Amcl Pose x'].mean()
                mean_x_amcl= round(mean_x_amcl, number_decimals)
                mean_y_amcl=filtered_data['Amcl Pose y'].mean()
                mean_y_amcl= round(mean_y_amcl, number_decimals)
                mean_x_gt=filtered_data['Ground Truth x'].mean()
                mean_x_gt= round(mean_x_gt, number_decimals)
                mean_y_gt=filtered_data['Ground Truth y'].mean()
                mean_y_gt= round(mean_y_gt, number_decimals)
                accuracy=error_position(mean_x_gt, mean_y_gt, mean_x_amcl, mean_y_amcl)
                accuracy=round(accuracy, number_decimals)


                std_deviation=filtered_data['Position Error'].std()
                std_deviation=round(std_deviation, number_decimals)


               
                #Repeatibility
                filtered_data['l'] = np.sqrt((filtered_data['Amcl Pose x'] - mean_x_amcl)**2 + (filtered_data['Amcl Pose y'] - mean_y_amcl)**2)
                mean_l=filtered_data['l'].mean() #Mean l
                l_std=filtered_data['l'].std()
                repeatibility=mean_l+ 2*l_std
                repeatibility=round(repeatibility, number_decimals)

                mean_error_positions.append(filtered_data['Position Error'].mean())
                mean_error_positions[x-1] = round(mean_error_positions[x-1], number_decimals)
                min_error_positions.append(filtered_data['Position Error'].min())
                min_error_positions[x-1] = round(min_error_positions[x-1], number_decimals)
                max_error_positions.append(filtered_data['Position Error'].max())
                max_error_positions[x-1] = round(max_error_positions[x-1], number_decimals)


                if mean_error_positions[x-1]>= biggest_mean_error:
                    biggest_mean_error=mean_error_positions[x-1]
                    a=x #Point with biggest mean error position

                cell = TableCell()
                cell.addElement(P(text=str(accuracy)))
                row_table2.addElement(cell)

                cell = TableCell()
                cell.addElement(P(text=str(repeatibility)))
                row_table2.addElement(cell)

                cell = TableCell()
                cell.addElement(P(text=str(mean_error_positions[x-1])))
                row_table2.addElement(cell)

                cell = TableCell()
                cell.addElement(P(text=str(min_error_positions[x-1])))
                row_table2.addElement(cell)

                cell = TableCell()       
                cell.addElement(P(text=str(max_error_positions[x-1])))
                row_table2.addElement(cell)

                cell = TableCell()       
                cell.addElement(P(text=str(std_deviation)))
                row_table2.addElement(cell)
            
            cell = TableCell()       
            cell.addElement(P(text=str(a)))
            row_table2.addElement(cell)


            table2.addElement(row_table2)

            ############################################# Table 3 ###############################################
            mean_error_orientations=[]
            min_error_orientations=[]
            max_error_orientations=[]
            row_table3 = TableRow()
            cell = TableCell()
            cell.addElement(P(text=str(dimension)))
            row_table3.addElement(cell)
            biggest_mean_error=0
            a=0 
            for x in range (1,6):
                filtered_data = data[data['Configuration number'] == x]
                mean_error_orientations.append(filtered_data['Orientation Error'].mean())
                mean_error_orientations[x-1] = round(mean_error_orientations[x-1], number_decimals)
                min_error_orientations.append(filtered_data['Orientation Error'].min())
                min_error_orientations[x-1] = round(min_error_orientations[x-1], number_decimals)
                max_error_orientations.append(filtered_data['Orientation Error'].max())
                max_error_orientations[x-1] = round(max_error_orientations[x-1], number_decimals)
                std_deviation=filtered_data['Orientation Error'].std()
                std_deviation=round(std_deviation, number_decimals)


                if mean_error_orientations[x-1]>= biggest_mean_error:
                    biggest_mean_error=mean_error_orientations[x-1]
                    a=x #Point with biggest mean error orientation

                #Radians to degrees
                mean_error_orientations[x-1]=math.degrees(mean_error_orientations[x-1])
                mean_error_orientations[x-1] = round(mean_error_orientations[x-1], number_decimals)
                min_error_orientations[x-1]=math.degrees(min_error_orientations[x-1])
                min_error_orientations[x-1] = round(min_error_orientations[x-1], number_decimals)
                max_error_orientations[x-1]=math.degrees(max_error_orientations[x-1])
                max_error_orientations[x-1] = round(max_error_orientations[x-1], number_decimals)



                

                cell = TableCell()
                cell.addElement(P(text=str(mean_error_orientations[x-1])))
                row_table3.addElement(cell)

                cell = TableCell()
                cell.addElement(P(text=str(min_error_orientations[x-1])))
                row_table3.addElement(cell)

                cell = TableCell()       
                cell.addElement(P(text=str(max_error_orientations[x-1])))
                row_table3.addElement(cell)

                cell = TableCell()       
                cell.addElement(P(text=str(std_deviation)))
                row_table3.addElement(cell)
            
            cell = TableCell()       
            cell.addElement(P(text=str(a)))
            row_table3.addElement(cell)

            table3.addElement(row_table3)

        
    elif data.empty:
        print('DataFrame empty ')

doc = load(file_path)
for sheet in doc.spreadsheet.getElementsByType(Table):
    sheet_name = sheet.getAttribute("name")
    if sheet_name == 'Accuracy' or sheet_name == 'Position Stadistics' or sheet_name == 'Orientation Stadistics' :
        doc.spreadsheet.removeChild(sheet)


doc.spreadsheet.addElement(table)
doc.spreadsheet.addElement(table2)
doc.spreadsheet.addElement(table3)
doc.save(file_path)
print("Acurracy calculation done")
