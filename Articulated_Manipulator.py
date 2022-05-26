import numpy as np
import math
import PySimpleGUI as sg
import pandas as pd

# GUI code

sg.theme('LightGreen10')

# Excel read code

EXCEL_FILE = '3-DOF Articulated Design Data.xlsx'
df = pd.read_excel(EXCEL_FILE)

# Layout code

layout = [
    [sg.Push()],
    
    [sg.Push(), sg.Text('Articulated Manipulator Design Calculator', font = ("Eras Demi ITC", 16)), sg.Push()],
    
    [sg.Push()],
    [sg.Push()],
    
    [sg.Text('Forward Kinematics Calculator', font = ("Eras Demi ITC", 11))],
    
    [sg.Push(), sg.Button('Click this before Solving Forward Kinematics', font = ("Eras Demi ITC", 13), size=(40,0), button_color=('white', 'red')), sg.Push()],
    
    [sg.Text('Fill out the following fields:', font = ("Eras Demi ITC", 12))],
    
    [sg.Text('a1 = ', font = ('Eras Demi ITC', 10)),sg.InputText('1', key='a1', size=(30,20)),
     sg.Text('T1 = ', font = ('Eras Demi ITC', 10)),sg.InputText('90', key='T1', size=(30,20)),
     sg.Push(), sg.Button('Jacobian Matrix (J)', font = ('Eras Demi ITC', 12), size = (30,0), button_color=('blue', 'pink')),
     sg.Button('Determinant (J)', font = ('Eras Demi ITC', 12), size = (30,0), button_color=('blue', 'pink'))],
    
    [sg.Text('a2 = ', font = ('Eras Demi ITC', 10)),sg.InputText('1', key='a2', size=(30,20)),
     sg.Text('T2 = ', font = ('Eras Demi ITC', 10)),sg.InputText('30', key='T2', size=(30,20)),
     sg.Push(), sg.Button('Inverse of J', font = ('Eras Demi ITC', 12), size = (30,0), button_color=('blue', 'pink')),
     sg.Button('Transpose of J', font = ('Eras Demi ITC', 12), size = (30,0), button_color=('blue', 'pink'))],
    
    [sg.Text('a3 = ', font = ('Eras Demi ITC', 10)),sg.InputText('1', key='a3', size=(30,20)),
     sg.Text('T3 = ', font = ('Eras Demi ITC', 10)),sg.InputText('60', key='T3', size=(30,20)), sg.Push(),
     sg.Button('Inverse Kinematics', font = ('Eras Demi ITC', 12), size=(62,0), button_color=('blue', 'pink'))],
    
    [sg.Button('Solve Forward Kinematics', font = ('Eras Demi ITC', 12), size=(53,0), button_color=('lightgreen', 'gray')),
     sg.Push(), sg.Button('Path and Trajectory Planning', font = ('Eras Demi ITC', 12), size =(62,0), button_color=('blue', 'pink'))],
    
    [sg.Frame('Position Vector: ',[[
        sg.Text('X = ', font = ('Eras Demi ITC', 10)),sg.InputText(key='X', size =(18,1)),
        sg.Text('Y = ', font = ('Eras Demi ITC', 10)),sg.InputText(key='Y', size =(18,1)),
        sg.Text('Z = ', font = ('Eras Demi ITC', 10)),sg.InputText(key='Z', size =(18,1)),]], font = ('Eras Demi ITC', 10))],
    
    [sg.Frame('H0_3 Transformation Matrix & Position Vectors = ',[[sg.Output(size=(70,20))]]),
     sg.Push(), sg.Image('Articulated.gif')],

    [sg.Submit(font = ('Eras Demi ITC', 12), button_color=('white', 'black')), sg.Exit(font = ('Eras Demi ITC', 12), button_color=('white', 'black'))]
    ]

# Window Code
window = sg.Window('Articulated Manipulator 3 DOF Forward Kinematics',layout, resizable=True)

# Variable Codes for disabling buttongs
disable_J = window['Jacobian Matrix (J)']
disable_DetJ = window['Determinant (J)']
disable_IV = window['Inverse of J']
disable_TJ = window['Transpose of J']
disable_IK = window['Inverse Kinematics']
disable_PT = window['Path and Trajectory Planning']


while True:
    event,values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    
    if event == 'Click this before Solving Forward Kinematics':
        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=True)
        disable_IV.update(disabled=True)
        disable_TJ.update(disabled=True)
        disable_IK.update(disabled=True)
        disable_PT.update(disabled=True)
    
    if event == 'Solve Forward Kinematics':
        # Forward Kinematic Codes
        # Link Lengths in mm
        a1 = values['a1']
        a2 = values['a2']
        a3 = values['a3']
        # Joint Variables in degrees
        T1 = values['T1']
        T2 = values['T2']
        T3 = values['T3']

        # Joint Variables in radians
        T1 = (float(T1)/180.0)*np.pi
        T2 = (float(T2)/180.0)*np.pi
        T3 = (float(T3)/180.0)*np.pi
        # If Joint Variable are ds don't need to convert
        ## D-H Parameter Table (This is the only part you need to edit for every new mechanical manipulator.)
        # Rows = no. of HTM, Columns - no. of Parameters
        # Theta, alpha, r, d
        DHPT = [[(0.0/180.0)*np.pi+float(T1),(90.0/180.0)*np.pi,0,float(a1)],
                [(0.0/180.0)*np.pi+float(T2),(0.0/180)*np.pi,float(a2),0],
                [(0.0/180.0)*np.pi+float(T3),(0.0/180)*np.pi,float(a3),0]]
        
        i = 0
        H0_1 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                [0,0,0,1]]
        i = 1
        H1_2 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                [0,0,0,1]]
        i = 2
        H2_3 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                [0,0,0,1]]
    
        H0_1 = np.matrix(H0_1)
        #print("H0_1=")
        #print(H0_1)

        H1_2 = np.matrix(H1_2)
        #print("H1_2=")
        #print(H1_2)

        H2_3 = np.matrix(H2_3)
        #print("H2_3=")
        #print(H2_3)

        H0_2 = np.dot(H0_1,H1_2)
        H0_3 = np.dot(H0_2,H2_3)
        print("H0_3=")
        print(np.matrix(H0_3))

        # Position Vectors X Y Z
        X0_3 = H0_3[0,3]
        print("X = ")
        print(X0_3)

        Y0_3 = H0_3[1,3]
        print("Y = ")
        print(Y0_3)

        Z0_3 = H0_3[2,3]
        print("Z = ")
        print(Z0_3)

        disable_J.update(disabled=False)
        disable_IK.update(disabled=False)
        disable_PT.update(disabled=False)
        
    if event == 'Submit':
        df = df.append(values, ignore_index=True)
        df.to_excel(EXCEL_FILE, index=False)
        sg.popup('Data Saved!')
        
    if event == 'Jacobian Matrix (J)':
        ### Jacobian Matrix
        ## 1. Linear/Prismatic Vectors
        Z_1 = [[0],[0],[1]]

        # Rows 1-3, Column 1
        J1a = [[1,0,0],[0,1,0],[0,0,1]]
        J1a = np.dot(J1a,Z_1)

        try:
            H0_3 = np.matrix(H0_3)
        except:
            H0_3 = -1 #NAN
            sg.popup('Warning!')
            sg.popup('Restart the GUI and click the TOP BUTTON first!')
            break
        
        J1b_1 = H0_3[0:3,3:] 
        #print("J1b_1 = ")
        #print(np.matrix(J1b_1)
        
        J1b_2 = [[0],[0],[0]]
        #print(np.matrix(J1b_2))
        
        J1b = J1b_1 - J1b_2
        #print('J1b = ')
        #print(np.matrix(J1b))

        J1 = [[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
              [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
              [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]
        #print('J1 = ')
        #print(np.matrix(J1))

        # Rows 1-3, Column 2
        try:
            H0_1 = np.matrix(H0_1)
        except:
            H0_1 = -1 #NAN
            sg.popup('Warning! The TOP BUTTON was not pressed!')
            sg.popup('Restart the GUI and click the TOP BUTTON first!')
            break
        
        J2a = H0_1[0:3,0:3]
        J2a = np.dot(J2a,Z_1)
        #print("J2a = ")
        #print(J2a)

        J2b_1 = H0_3[0:3,3:]
        J2b_1 = np.matrix(J2b_1)
        #print("J2b_1 = ")
        #print(J2b_1)

        J2b_2 = H0_1[0:3,3:]
        J2b_2 = np.matrix(J2b_2)
        #print("J2b_2 = ")
        #print(J2b_2)

        J2b = J2b_1 - J2b_2
        #print("J2b = ")
        #print(J2b)

        J2 = [[(J2a[1,0]*J2b[2,0])-(J2a[2,0]*J2b[1,0])],
              [(J2a[2,0]*J2b[0,0])-(J2a[0,0]*J2b[2,0])],
              [(J2a[0,0]*J2b[1,0])-(J2a[1,0]*J2b[0,0])]]
        #print("J2 = ")
        #print(np.matrix(J2))

        # Rows 1-3, Column 3
        J3a = H0_2[0:3,0:3]
        J3a = np.dot(J3a,Z_1)
        #print("J3a = ")
        #print(J3a)

        J3b_1 = H0_3[0:3,3:]
        J3b_1 = np.matrix(J3b_1)
        #print("J3b_1 = ")
        #print(J3b_1)

        J3b_2 = H0_2[0:3,3:]
        J3b_2 = np.matrix(J3b_2)
        #print("J3b_2 = ")
        #print(J3b_2)

        J3b = J3b_1 - J3b_2
        #print("J3b = ")
        #print(J3b)
        J3 = [[(J3a[1,0]*J3b[2,0])-(J3a[2,0]*J3b[1,0])],
              [(J3a[2,0]*J3b[0,0])-(J3a[0,0]*J3b[2,0])],
              [(J3a[0,0]*J3b[1,0])-(J3a[1,0]*J3b[0,0])]]
        #print("J3 = ")
        #print(np.matrix(J3))

        ## 2. Rotation/Orientation Vectors
        J4 = [[0],[0],[1]]
        J4 = np.matrix(J4)
        #print("J4 = ")
        #print(J4)

        J5 = H0_1[0:3,0:3]
        J5 = np.dot(J5,Z_1)
        J5 = np.matrix(J5)
        #print("J5 = ")
        #print(J5)
         
        J6 = H0_2[0:3,0:3]
        J6 = np.dot(J6,Z_1)
        J6 = np.matrix(J6)
        #print("J6 = ")
        #print(J6)
    
        JM1 = np.concatenate((J1,J2,J3),1)
        #print(JM1)
        JM2 = np.concatenate((J1a,J2a,J3a),1)
        #print(JM2)

        J = np.concatenate((JM1, JM2),0)
        #print("J = ")
        #print(J)

        sg.popup('J = ', J)

        disable_J.update(disabled =True)
        disable_DetJ.update(disabled =False)
        disable_IV.update(disabled =False)
        disable_TJ.update(disabled =False)

    if event == 'Determinant (J)':
        #singularity =Det(J)
        #np.linalg.det(M)
        #Let JM1 become the 3X3 position matrix for obtaining the Determinant

        try:
            JM1 = np.concatenate((J1,J2,J3),1)
        except:
            JM1 = -1 #NAN
            sg.popup('Warning!')
            sg.popup('Restart the GUI and click the TOP BUTTON first!')
            break

        DJ = np.linalg.det(JM1)
        #print("DJ = ", DJ)

        sg.popup('DJ = ',DJ)

        if DJ == 0.0 or DJ == -0.0:
            disable_IV.update(disabled =True)
            sg.popup('Warning: Jacobian Matrix is Non-invertable!')

    if event == 'Inverse of J':
        #Inv(J)
    
        try:
            JM1 = np.concatenate((J1,J2,J3),1)
        except:
            JM1 = -1 #NAN
            sg.popup('Warning!')
            sg.popup('Restart the GUI and click the TOP BUTTON first!')
            break
        
        IJ =np.linalg.inv(JM1)
        #print("IV = ")
        #print(IV)
        sg.popup('IJ = ', IJ)

    if event == 'Transpose of J':

        try:
            JM1 = np.concatenate((J1,J2,J3),1)
        except:
            JM1 = -1 #NAN
            sg.popup('Warning!')
            sg.popup('Restart the GUI and click the TOP BUTTON first!')
            break

        TJ = np.transpose(JM1)
        #print("TJ = ")
        #print(TJ)
        
        sg.popup('TJ = ', TJ)

window.close() 


