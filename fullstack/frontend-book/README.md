# AI-Spec Driven Book - Frontend

A modern, accessible learning platform for Physical AI and Humanoid Robotics education. Built with Docusaurus 3.9.2 and React 19, this platform provides comprehensive educational content covering the full humanoid autonomy pipeline from ROS 2 fundamentals to Vision-Language-Action systems.

**Live Platform**: https://ai-spec-driven-book-six.vercel.app/

## Overview

The AI-Spec Driven Book is a fullstack educational platform teaching Physical AI and Humanoid Robotics through spec-driven development. The frontend delivers 6 comprehensive learning modules with an embedded RAG-powered chatbot for instant assistance.

### Core Mission
Bridge the gap between digital AI and embodied intelligence by providing a comprehensive curriculum covering robotics middleware, kinematics, simulation, perception systems, NVIDIA Isaac, and VLA systems.

## Features

- **6 Comprehensive Modules**: Progressive learning path from ROS 2 basics to advanced VLA systems
- **Embedded RAG Chatbot**: AI-powered assistant with conversational context management
- **Accessibility-First**: WCAG 2.1 AA compliant with semantic HTML, ARIA labels, keyboard navigation
- **Internationalization**: English + Urdu with RTL support
- **Dark/Light Mode**: System-aware theme switching with smooth transitions
- **Responsive Design**: Optimized for mobile, tablet, and desktop
- **Performance Optimized**: Code splitting, lazy loading, CDN delivery via Vercel

## Getting Started

### Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control

### Tech Stack

- **Framework**: Docusaurus 3.9.2 (React-based static site generator)
- **UI Library**: React 19.0.0
- **Animation**: Framer Motion 12.34.0
- **Icons**: Lucide React 0.562.0
- **Styling**: CSS Modules + Custom CSS
- **Deployment**: Vercel (with automatic CI/CD)

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd frontend-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

### Development

To start the development server:
```bash
npm start
```

This will start a local development server at `http://localhost:3000` with hot reloading.

### Building for Production

To build the static site for production:
```bash
npm run build
```

The built site will be available in the `build/` directory.

### Serving the Built Site

To serve the built site locally for testing:
```bash
npm run serve
```

## Project Structure

```
frontend-book/
├── docs/                      # Educational content (6 modules)
│   ├── module-1/             # Robotic Nervous System
│   ├── module-2/             # Robot Kinematics & Structure
│   ├── module-3/             # Digital Twin Simulation
│   ├── module-4/             # Perception Systems
│   ├── module-5/             # AI-Robot Brain (NVIDIA Isaac)
│   └── module-6/             # Vision-Language-Action (VLA)
├── src/
│   ├── components/           # React components
│   │   ├── Chatbot/         # RAG chatbot integration
│   │   ├── UI/              # Reusable UI components
│   │   └── Theme/           # Theme system components
│   ├── css/                 # Global styles
│   └── pages/               # Custom pages
├── static/                   # Static assets (images, icons)
├── docusaurus.config.js     # Main configuration
├── sidebars.js              # Navigation structure
└── package.json             # Dependencies
```

## Learning Modules

### Module 1: Robotic Nervous System
- ROS 2 fundamentals and middleware architecture
- Core communication concepts (nodes, topics, services)
- AI agents and robot models integration

### Module 2: Robot Kinematics & Structure
- Links, joints, and coordinate frames
- Forward and inverse kinematics
- URDF mapping between real and simulated robots
- Joint constraints and motion limits

### Module 3: Digital Twin Simulation
- Gazebo simulation setup and configuration
- Unity visualization integration
- Physics and collision modeling
- Navigation and motion planning

### Module 4: Perception Systems
- Robot camera models and calibration
- LiDAR fundamentals and processing
- IMU sensor fusion techniques
- Building complete perception pipelines

### Module 5: AI-Robot Brain (NVIDIA Isaac)
- Isaac Sim overview and architecture
- Synthetic data generation for training
- Hardware-accelerated VSLAM with Isaac ROS
- Nav2 path planning for humanoid mobile robots

### Module 6: Vision-Language-Action (VLA)
- VLA fundamentals and architecture
- Voice-to-action system implementation
- Cognitive planning with language models
- Executing language-based action plans

## Contributing

To contribute content or improvements:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Key Components

### Chatbot Integration
- **ChatContext.jsx**: State management for conversation history
- **ChatPanel.jsx**: Main chat interface with message rendering
- **ChatMessage.jsx**: Individual message component with citations
- **useChatApi.js**: API integration with backend RAG service

### UI Components
- **AccessibleButton**: WCAG-compliant button component
- **Card, FeatureCard**: Content presentation components
- **LearningObjectives, LearningOutcomes**: Educational components
- **IsaacHighlight**: Specialized component for NVIDIA Isaac content

### Theme System
- **ThemeToggleButton**: Dark/light mode switcher
- **ThemeSystem**: System-aware theme management
- Smooth transitions between themes
- Persistent user preferences

## Deployment

This site is deployed to Vercel with automatic CI/CD:
- **Production**: https://ai-spec-driven-book-six.vercel.app/
- **Preview**: Automatic preview deployments for pull requests
- **CDN**: Global content delivery for optimal performance

## Accessibility

WCAG 2.1 AA compliant with:
- Semantic HTML structure
- ARIA labels and roles
- Keyboard navigation support
- Screen reader optimization
- High contrast ratios (4.5:1 minimum)
- Focus indicators on all interactive elements

## Performance

- Code splitting for faster initial loads
- Lazy loading for images and heavy components
- Optimized asset delivery via Vercel CDN
- Minimal bundle size with tree shaking
- Service worker for offline support

## License

This project is licensed under the MIT License.
