import { betterAuth } from "better-auth";
import pg from "pg";

// PostgreSQL connection string - in production, this should come from environment variables
const connectionString = process.env.DATABASE_URL || "";

export const auth = betterAuth({
  database: {
    type: "postgres",
    connection: new pg.Pool({
      connectionString: connectionString,
    }),
    // Custom user schema to store software and hardware background
    schema: {
      user: {
        fields: {
          // Add custom fields for user's software and hardware background
          softwareExperience: "string",      // User's software experience level (beginner/intermediate/advanced)
          hardwareAccess: "string",          // What hardware does the user have access to
          programmingLanguages: "string",    // Which programming languages the user knows
          primaryFocus: "string",            // Primary focus area (software/hardware/both)
          projectGoals: "string",            // What projects the user wants to work on
          skillLevel: "string",              // Overall skill level (beginner/intermediate/advanced)
        },
      },
    },
  },
  // Enable email/password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // For MVP, we won't require email verification
  },
  // Session configuration
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60,     // Update session every 24 hours
  },
  // Account configuration
  account: {
    accountLinking: {
      enabled: true,
    },
  },
  // Social providers can be added here later if needed
  socialProviders: {},
  // UI configuration
  ui: {
    enabled: true,
  },
});

// Define the custom user attributes type for TypeScript
export interface CustomUserAttributes {
  softwareExperience?: string;
  hardwareAccess?: string;
  programmingLanguages?: string;
  primaryFocus?: string;
  projectGoals?: string;
  skillLevel?: string;
}
