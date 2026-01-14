#!/bin/bash

echo "=========================================="
echo "Setting up Authentication System"
echo "=========================================="
echo ""

# Navigate to backend
cd "$(dirname "$0")/backend" || exit 1

echo "✓ Current directory: $(pwd)"
echo ""

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "❌ Failed to install dependencies"
    exit 1
fi
echo "✓ Dependencies installed successfully"
echo ""

# Run database migrations
echo "🗄️  Running database migrations..."
python3 -m alembic upgrade head
if [ $? -ne 0 ]; then
    echo "❌ Failed to run migrations"
    exit 1
fi
echo "✓ Database tables created successfully"
echo ""

# Start the backend server
echo "=========================================="
echo "🚀 Starting Backend Server..."
echo "=========================================="
echo ""
echo "Backend will be available at: http://localhost:8000"
echo "API docs available at: http://localhost:8000/docs"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

python3 -m uvicorn src.main:app --reload --port 8000
