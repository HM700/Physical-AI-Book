# Physical AI & Humanoid Robotics Book

A comprehensive educational resource for Physical AI and Humanoid Robotics, built with Docusaurus.

## 📘 About This Book

This book is designed for students, researchers, and engineers interested in understanding and developing AI-powered physical systems and humanoid robots. Through a combination of theoretical foundations and practical examples, you'll learn how to build sophisticated robotic systems that can perceive, reason, and act in the physical world.

## 📚 Book Structure

The book is organized into several modules:

- **Module 1**: ROS 2 Fundamentals - Learn the Robot Operating System and its role in robotics development
- **Module 2**: Simulation Environments - Master Gazebo and Unity for robot simulation
- **Module 3**: Isaac AI Framework - Develop perception and control systems using NVIDIA Isaac
- **Module 4**: Vision Language Action (VLA) Models - Implement advanced AI models for humanoid robots

## 🚀 Quick Start

### Installation

```bash
npm install
```

### Development

```bash
npm start
```

This will start the development server at `http://localhost:3000`.

### Build

```bash
npm run build
```

This will generate the static files in the `build` directory.

## 📁 Project Structure

```
├── docs/                   # Book content
│   ├── chapters/          # Book modules and chapters
│   │   ├── intro.md      # Introduction to the book
│   │   ├── module-1/     # ROS 2 Fundamentals
│   │   ├── module-2/     # Simulation Environments
│   │   ├── module-3/     # Isaac AI Framework
│   │   └── module-4/     # VLA Models
│   ├── examples/          # Code examples and exercises
│   ├── reference/         # Reference materials
│   └── assets/            # Image and other assets
├── src/                   # Custom React components and styles
│   ├── components/        # Reusable React components
│   ├── css/              # Custom styles
│   └── pages/            # Additional pages
├── docusaurus.config.ts   # Docusaurus configuration
├── sidebars.ts           # Navigation sidebar configuration
└── package.json          # Dependencies and scripts
```

## 🛠️ Technologies Used

- [Docusaurus](https://docusaurus.io/) - Static site generator for documentation
- [React](https://reactjs.org/) - Component-based UI library
- [TypeScript](https://www.typescriptlang.org/) - Type-safe JavaScript
- [Node.js](https://nodejs.org/) - JavaScript runtime

## 🚀 Deployment

### Deploy to Vercel

1. Fork this repository
2. Connect your fork to Vercel
3. Choose the following settings:
   - Framework preset: Docusaurus
   - Build command: `npm run build`
   - Output directory: `build`
   - Install command: `npm install`

### Deploy to GitHub Pages

1. Enable GitHub Pages in your repository settings
2. Select the `gh-pages` branch or `/docs` folder as the source
3. Run `npm run deploy` to build and deploy to GitHub Pages

### Environment Variables

No special environment variables are required for this project.

## 🤝 Contributing

We welcome contributions to improve the book content and examples. To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

If you have questions or need help, please open an issue in the GitHub repository.
