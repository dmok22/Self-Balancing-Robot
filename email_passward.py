from tkinter import simpledialog
from tkinter import messagebox
from tkinter import ttk
import tkinter as tk
import secrets
import string
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import sys


def get_user_input(prompt):
    root = tk.Tk()
    root.withdraw()

    user_input = simpledialog.askstring("Input", prompt)

    return user_input

def generate_random_number_password(length=6):
    # Use only digits for the password
    digits = string.digits
    password = ''.join(secrets.choice(digits) for i in range(length))
    return password

def send_email(subject, message, to_email):
    from_email = "20040418jeff@gmail.com"
    password = "bxmu rzjz ppah gjsd"

    # Create the email message
    msg = MIMEMultipart()
    msg['From'] = from_email
    msg['To'] = to_email
    msg['Subject'] = subject
    body = MIMEText(message, 'plain')
    msg.attach(body)

    # Send the email
    with smtplib.SMTP('smtp.gmail.com', 587) as server:
        server.starttls()
        server.login(from_email, password)
        text = msg.as_string()
        server.sendmail(from_email, to_email, text)


password = generate_random_number_password()

#get email
ask_email = "Please Enter The Email Where You Want To Get Your Password:"
email = get_user_input(ask_email)

#end email
send_email("Oven Controller Password", f"Your password is: {password}", email)

password_chance = 3
while password_chance >0:
    get_password = f"please enter the password sent to your email:\nyou have {password_chance} chances."
    email_password = get_user_input(get_password)
    if email_password == password:
        displace_message("password correct :)")
        break
    else:
        displace_message("password wrong :(")
        password_chance = password_chance - 1

if password_chance == 0:
    displace_message("all attempt used, please restart the program")
    sys.exit()
