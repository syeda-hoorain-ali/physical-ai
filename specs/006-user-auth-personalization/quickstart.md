# Quickstart Guide: User Authentication with Personalized Content

## Prerequisites
- Node.js 20+ installed
- PostgreSQL database setup
- Docusaurus project running

## Setup Steps

### 1. Install Dependencies
```bash
npm install better-auth @better-auth/client @better-auth/react
```

### 2. Configure Environment Variables
Create/update `.env.local`:
```bash
DATABASE_URL=postgresql://username:password@localhost:5432/physical-ai
BETTER_AUTH_SECRET=your-super-secret-key-here
SITE_URL=http://localhost:3000
```

### 3. Create Auth Configuration
Create `book-source/src/lib/auth.ts`:
```typescript
import { betterAuth } from 'better-auth';

const customUserAttributes = {
  softwareExperience: { type: 'string', required: false },
  hardwareAccess: { type: 'string', required: false },
  programmingLanguages: { type: 'string', required: false },
  hardwareExperience: { type: 'string', required: false },
  learningGoals: { type: 'string', required: false },
  educationalBackground: { type: 'string', required: false },
  timeCommitment: { type: 'string', required: false },
  aiMlExperience: { type: 'string', required: false },
  rosExperience: { type: 'string', required: false },
  simulationExperience: { type: 'string', required: false },
};

export const auth = betterAuth({
  database: {
    provider: 'postgresql',
    url: process.env.DATABASE_URL || 'postgresql://localhost:5432/physical-ai',
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  user: {
    additionalFields: customUserAttributes,
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    rememberMe: true,
  },
  baseURL: process.env.SITE_URL || 'http://localhost:3000',
  secret: process.env.BETTER_AUTH_SECRET || 'your-secret-key-for-development',
});
```

### 4. Create Auth Provider Component
Create `book-source/src/components/auth/auth-provider.tsx`:
```tsx
import React from 'react';
import { AuthProvider as BetterAuthProvider } from 'better-auth/react';
import { auth } from '../../lib/auth';

interface AuthProviderProps {
  children: React.ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  return <BetterAuthProvider value={auth}>{children}</BetterAuthProvider>;
};
```

### 5. Wrap Your App
Update `book-source/src/theme/Layout.tsx` to wrap the app with the AuthProvider.

### 6. Create Custom Signup Form
Create `book-source/src/components/auth/signup-form.tsx` with background questions.

### 7. Run Database Migrations
```bash
npx better-auth migrate
```

## Environment-Specific Configuration

### Development
- Use local PostgreSQL instance
- Set SITE_URL to `http://localhost:3000`

### Production
- Use production PostgreSQL database
- Set appropriate SITE_URL
- Use strong BETTER_AUTH_SECRET
