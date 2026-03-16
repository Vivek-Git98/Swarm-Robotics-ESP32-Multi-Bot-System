# 📤 How to Upload This Project to GitHub

A complete step-by-step guide — even if you've never used Git before.

---

## Step 1 — Install Git

- **Windows**: Download from https://git-scm.com → Install with defaults
- **Linux/Mac**: `sudo apt install git` or `brew install git`

Verify: open a terminal and type `git --version`

---

## Step 2 — Create a GitHub Account

Go to https://github.com → Sign up (free account is fine)

---

## Step 3 — Create a New Repository on GitHub

1. Click the **+** icon (top right) → **New repository**
2. Fill in:
   - **Name**: `swarm-robotics`
   - **Description**: `ESP32 three-bot swarm system with Python GUI`
   - Set to **Public** (so people can see it)
   - ✅ Check **Add a README file** → *uncheck* this since we already have one
3. Click **Create repository**
4. Copy the repository URL shown (looks like `https://github.com/YourUsername/swarm-robotics.git`)

---

## Step 4 — Set Up Git on Your PC

Open terminal (Command Prompt / PowerShell on Windows):

```bash
git config --global user.name  "Your Name"
git config --global user.email "your@email.com"
```

---

## Step 5 — Initialize and Push the Project

Navigate to the project folder (where you downloaded these files):

```bash
cd path/to/swarm-robotics

git init
git add .
git commit -m "Initial commit: ESP32 swarm firmware + Python GUI"
git branch -M main
git remote add origin https://github.com/YourUsername/swarm-robotics.git
git push -u origin main
```

> Replace `YourUsername` with your actual GitHub username.

When prompted, enter your GitHub **username** and a **Personal Access Token**  
(not your password — GitHub requires tokens now).

### How to get a Personal Access Token:
1. GitHub → Settings → Developer Settings → Personal Access Tokens → Tokens (classic)
2. Click **Generate new token** → check `repo` scope → Generate
3. Copy it immediately (you won't see it again) and use it as your password

---

## Step 6 — Add Your Photos

1. Put your bot photos in the `media/photos/` folder
   - Recommended names: `bot_overview.jpg`, `bot_side.jpg`, `formation.jpg`
2. Run:
```bash
git add media/photos/
git commit -m "Add bot photos"
git push
```

> **Photo tips**: Crop to show the bot clearly. 1200×800px or larger is ideal.  
> The README will display them automatically.

---

## Step 7 — Add Your Demo Video

GitHub has a **100 MB file limit**. Videos are usually too large, so use one of these options:

### Option A — GitHub Releases (recommended for large videos)
1. On GitHub repo page → click **Releases** (right sidebar) → **Create a new release**
2. Tag: `v1.0` | Title: `Demo Video`
3. Drag your video file into the **Attach binaries** area
4. Publish release
5. Copy the video URL and update README.md:
```markdown
🎥 **[Watch Demo Video](https://github.com/YourUsername/swarm-robotics/releases/download/v1.0/swarm_demo.mp4)**
```

### Option B — YouTube (easiest)
1. Upload to YouTube (can be unlisted)
2. Replace the video line in README.md:
```markdown
🎥 **[Watch Demo Video on YouTube](https://youtu.be/YOUR_VIDEO_ID)**
```

### Option C — Small video (<25MB)
If your video is small, just commit it normally:
```bash
git add media/videos/swarm_demo.mp4
git commit -m "Add demo video"
git push
```

---

## Step 8 — Update README with Your Details

Open `README.md` and fill in:
- Your name and college in the **Author** section
- Your LinkedIn / email
- Any additional context about the project

Then push the update:
```bash
git add README.md
git commit -m "Update author info"
git push
```

---

## Step 9 — Make It Look Professional (Optional)

### Add Topics / Tags
On your GitHub repo page → click the ⚙️ gear next to **About** → add topics:
```
esp32  swarm-robotics  esp-now  arduino  python  pid-control  robotics
```

### Enable GitHub Pages (optional project site)
Settings → Pages → Source: `main` branch → `/docs` folder

---

## Useful Git Commands Reference

```bash
git status                  # see what's changed
git add .                   # stage all changes
git commit -m "message"     # save a snapshot
git push                    # upload to GitHub
git pull                    # download latest from GitHub
git log --oneline           # see commit history
```

---

## Final Checklist

- [ ] All `.ino` files pushed  
- [ ] Python GUI pushed  
- [ ] README has your name and description  
- [ ] At least 2–3 photos uploaded  
- [ ] Demo video linked (YouTube or Release)  
- [ ] Repository is set to Public  
- [ ] Topics/tags added  

---

Your repo will be live at:  
**`https://github.com/YourUsername/swarm-robotics`**
