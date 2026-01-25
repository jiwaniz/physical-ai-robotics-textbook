"""
Email service for sending verification emails via Resend.
"""

import logging
import secrets
from datetime import datetime, timedelta
from typing import Optional

import resend

from ..core.config import settings

logger = logging.getLogger(__name__)

# Token expiry time (24 hours)
VERIFICATION_TOKEN_EXPIRY_HOURS = 24


def generate_verification_token() -> str:
    """Generate a secure random verification token."""
    return secrets.token_urlsafe(32)


def get_token_expiry() -> datetime:
    """Get the expiry time for a verification token."""
    return datetime.utcnow() + timedelta(hours=VERIFICATION_TOKEN_EXPIRY_HOURS)


async def send_verification_email(
    to_email: str,
    user_name: str,
    verification_token: str,
) -> bool:
    """
    Send a verification email to the user.

    Args:
        to_email: Recipient email address
        user_name: User's name for personalization
        verification_token: The verification token

    Returns:
        True if email was sent successfully, False otherwise
    """
    if not settings.resend_api_key:
        logger.warning("Resend API key not configured, skipping email send")
        return False

    resend.api_key = settings.resend_api_key

    verification_url = f"{settings.frontend_url}/verify-email?token={verification_token}"

    html_content = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <title>Verify Your Email</title>
        <style>
            body {{
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
                line-height: 1.6;
                color: #333;
                max-width: 600px;
                margin: 0 auto;
                padding: 20px;
            }}
            .button {{
                display: inline-block;
                background-color: #2563eb;
                color: white !important;
                text-decoration: none;
                padding: 12px 24px;
                border-radius: 6px;
                margin: 20px 0;
            }}
            .footer {{
                margin-top: 30px;
                padding-top: 20px;
                border-top: 1px solid #eee;
                font-size: 14px;
                color: #666;
            }}
        </style>
    </head>
    <body>
        <h1>Welcome to Physical AI & Robotics Textbook!</h1>
        <p>Hi {user_name},</p>
        <p>Thank you for signing up! Please verify your email address
        by clicking the button below:</p>
        <a href="{verification_url}" class="button">Verify Email Address</a>
        <p>Or copy and paste this link into your browser:</p>
        <p><a href="{verification_url}">{verification_url}</a></p>
        <p>This link will expire in {VERIFICATION_TOKEN_EXPIRY_HOURS} hours.</p>
        <div class="footer">
            <p>If you didn't create an account, you can safely ignore this email.</p>
            <p>Physical AI & Humanoid Robotics Textbook</p>
        </div>
    </body>
    </html>
    """

    try:
        params = {
            "from": settings.email_from,
            "to": [to_email],
            "subject": "Verify your email address",
            "html": html_content,
        }

        email = resend.Emails.send(params)
        logger.info(f"Verification email sent to {to_email}, id: {email.get('id')}")
        return True

    except Exception as e:
        logger.error(f"Failed to send verification email to {to_email}: {e}")
        return False


def is_token_expired(token_expires: Optional[datetime]) -> bool:
    """Check if a verification token has expired."""
    if token_expires is None:
        return True
    return datetime.utcnow() > token_expires
