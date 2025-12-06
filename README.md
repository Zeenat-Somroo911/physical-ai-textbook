# Physical AI & Humanoid Robotics Textbook

A comprehensive digital textbook project for Physical AI and Humanoid Robotics, featuring interactive content, code examples, and an AI-powered chatbot assistant.

## Project Overview

This project is designed as a collaborative hackathon effort to create an educational resource covering:

- **Physical AI**: The intersection of artificial intelligence and physical systems
- **Humanoid Robotics**: Design, control, and applications of humanoid robots
- **Interactive Learning**: Code examples, simulations, and hands-on exercises
- **AI Assistant**: Intelligent chatbot to help students learn and explore concepts

## Project Structure

```
physical-ai-textbook/
├── docs/                 # Textbook content and documentation
├── chatbot-backend/      # FastAPI backend for AI chatbot
├── src/                  # React components and frontend code
├── code-examples/        # Sample code and implementations
└── README.md            # This file
```

## Technology Stack

- **Frontend**: React + Docusaurus (documentation framework)
- **Backend**: FastAPI (Python web framework)
- **AI/ML**: Integration with AI models for chatbot functionality
- **Version Control**: Git

## Getting Started

### Prerequisites

- Node.js (v16 or higher)
- Python 3.8+
- Git

### Installation

1. Clone the repository:
```bash
git clone https://github.com/Zeenat-Somroo911/physical-ai-textbook.git
cd physical-ai-textbook
```

2. Install frontend dependencies:
```bash
npm install
```

3. Set up backend (see chatbot-backend/README.md for details):
```bash
cd chatbot-backend
pip install -r requirements.txt
```

4. Start the development server:
```bash
npm start
```

## 🚀 Deployment

### Frontend (GitHub Pages)

The Docusaurus frontend is automatically deployed to GitHub Pages on every push to `main` branch.

**Live Site:** [https://Zeenat-Somroo911.github.io/physical-ai-textbook/](https://Zeenat-Somroo911.github.io/physical-ai-textbook/)

**Manual Deployment:**
```bash
npm run build
npm run deploy:gh-pages
```

**Configuration:**
- GitHub Pages URL: `https://Zeenat-Somroo911.github.io/physical-ai-textbook/`
- Base URL: `/physical-ai-textbook/`
- Auto-deployment via GitHub Actions (`.github/workflows/deploy.yml`)

### Backend (Vercel)

The FastAPI backend can be deployed to Vercel for serverless hosting.

**Prerequisites:**
1. Vercel account: [vercel.com](https://vercel.com)
2. Vercel CLI: `npm i -g vercel`

**Deployment Steps:**

1. **Install Vercel CLI:**
   ```bash
   npm i -g vercel
   ```

2. **Login to Vercel:**
   ```bash
   vercel login
   ```

3. **Deploy from backend directory:**
   ```bash
   cd chatbot-backend
   vercel
   ```

4. **Set Environment Variables in Vercel Dashboard:**
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `QDRANT_URL` - Qdrant vector database URL
   - `DATABASE_URL` - Neon Postgres connection string
   - `JWT_SECRET` - Secret key for JWT tokens
   - `CORS_ORIGINS` - Allowed CORS origins (comma-separated)

5. **Production Deployment:**
   ```bash
   vercel --prod
   ```

**Vercel Configuration:**
- Config file: `chatbot-backend/vercel.json`
- Runtime: Python 3.10+
- Max duration: 30 seconds

### Environment Variables

#### Frontend (Optional)
No environment variables needed for frontend. All configuration is in `docusaurus.config.js`.

#### Backend (Required)

Create `.env` file in `chatbot-backend/` directory:

```env
# OpenAI Configuration
OPENAI_API_KEY=your-openai-api-key-here
OPENAI_MODEL=gpt-3.5-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key

# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@host:5432/database

# Authentication
JWT_SECRET=your-secret-key-here-min-32-chars
JWT_ALGORITHM=HS256
JWT_EXPIRATION=7d

# CORS
CORS_ORIGINS=http://localhost:3000,https://zeenat-somroo911.github.io

# Application
APP_NAME=Physical AI Textbook Chatbot
APP_VERSION=1.0.0
LOG_LEVEL=INFO
```

**Copy from example:**
```bash
cd chatbot-backend
cp .env.example .env
# Edit .env with your actual values
```

### Database Setup

1. **Create Neon Postgres Database:**
   - Sign up at [neon.tech](https://neon.tech)
   - Create a new project
   - Copy the connection string

2. **Run Schema:**
   ```bash
   cd chatbot-backend
   psql $DATABASE_URL -f schema.sql
   ```
   Or use Neon's SQL editor to run `schema.sql`

3. **Verify Tables:**
   ```sql
   \dt  -- List all tables
   ```

### Qdrant Setup

**Option 1: Local Qdrant (Development)**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

**Option 2: Qdrant Cloud (Production)**
1. Sign up at [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a cluster
3. Get API key and URL

## 📍 Live Links

### Production URLs

- **Frontend (GitHub Pages):** [https://Zeenat-Somroo911.github.io/physical-ai-textbook/](https://Zeenat-Somroo911.github.io/physical-ai-textbook/)
- **Backend API:** [Your Vercel URL]/api
- **GitHub Repository:** [https://github.com/Zeenat-Somroo911/physical-ai-textbook](https://github.com/Zeenat-Somroo911/physical-ai-textbook)

### Development URLs

- **Frontend (Local):** http://localhost:3000
- **Backend (Local):** http://localhost:8000
- **API Docs:** http://localhost:8000/docs

## 🔧 Troubleshooting Deployment

### GitHub Pages Issues

**Problem: 404 on all pages**
- **Solution:** Check `baseUrl` in `docusaurus.config.js` matches your repository name

**Problem: Assets not loading**
- **Solution:** Ensure `trailingSlash: false` in config

**Problem: Build fails**
- **Solution:** Check Node.js version (requires 18+), verify all dependencies installed

### Vercel Deployment Issues

**Problem: Import errors**
- **Solution:** Ensure `requirements.txt` has all dependencies

**Problem: Environment variables not working**
- **Solution:** Set variables in Vercel dashboard, not just `.env` file

**Problem: Database connection fails**
- **Solution:** Check `DATABASE_URL` format, ensure database allows connections from Vercel IPs

### Common Issues

**CORS Errors:**
- Add your frontend URL to `CORS_ORIGINS` in backend environment variables

**Authentication Not Working:**
- Verify `JWT_SECRET` is set and same across all environments
- Check token expiration settings

**Vector Search Not Working:**
- Verify Qdrant is running and accessible
- Check `QDRANT_URL` and `QDRANT_API_KEY` are correct

## 📝 Deployment Checklist

### Before First Deployment

- [ ] Update `docusaurus.config.js` with correct URLs
- [ ] Set all environment variables in Vercel
- [ ] Run database schema (`schema.sql`)
- [ ] Test backend locally
- [ ] Test frontend build locally (`npm run build`)
- [ ] Verify GitHub Actions workflow is set up
- [ ] Check CORS origins include production URL

### After Deployment

- [ ] Verify frontend loads correctly
- [ ] Test authentication flow
- [ ] Test chatbot functionality
- [ ] Test personalization feature
- [ ] Test translation feature
- [ ] Check API endpoints are accessible
- [ ] Monitor error logs

## Contributing

This is a hackathon project. Contributions are welcome! Please follow the project structure and coding standards.

## License

[To be determined]

## Contact

- **GitHub:** [@Zeenat-Somroo911](https://github.com/Zeenat-Somroo911)
- **Repository:** [physical-ai-textbook](https://github.com/Zeenat-Somroo911/physical-ai-textbook)

