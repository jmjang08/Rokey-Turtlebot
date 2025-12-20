# login_manager.py
from flask import session, redirect, request, flash

USERNAME = "admin"
PASSWORD = "1234"


def check_login():
    """Check if you are logged in"""
    if "username" not in session:
        return False
    return True


def try_login():
    """Login Attempt Handling"""
    username = request.form.get("username")
    password = request.form.get("password")

    if username == USERNAME and password == PASSWORD:
        session["username"] = username
        return True

    flash("Failed to login", "danger")
    return False


def logout_user():
    """logout"""
    session.clear()
