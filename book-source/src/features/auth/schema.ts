import z from "zod";

// Define the Zod schema for sign in form validation
export const signInSchema = z.object({
    email: z.email('Please enter a valid email address'),
    password: z.string().min(1, 'Password is required'),
});

export type SignInFormData = z.infer<typeof signInSchema>;


// Define the Zod schema for sign up form validation
export const signUpSchema = z.object({
    email: z.email('Please enter a valid email address'),

    password: z.string().min(8, 'Password must be at least 8 characters long'),

    name: z.string().min(1, 'Name is required'),

    softwareExperience: z.enum(['beginner', 'intermediate', 'advanced'], {
        error: 'Please select your software experience level',
    }),

    hardwareAccess: z.string().min(1, 'Please describe the hardware you have access to'),

    programmingLanguages: z.string().min(1, 'Please list programming languages you know'),

    primaryFocus: z.enum(['software', 'hardware', 'both'], {
        error: 'Please select your primary focus area',
    }),

    projectGoals: z.string().min(1, 'Please describe your project goals'),

    skillLevel: z.enum(['beginner', 'intermediate', 'advanced'], {
        error: 'Please select your overall skill level',
    }),
});

export type SignUpFormData = z.infer<typeof signUpSchema>;


// Define the Zod schema for user profile form validation
export const userProfileSchema = z.object({
    fullName: z.string(),
    email: z.email(),
    programmingLevel: z.enum(['none', 'beginner', 'intermediate', 'advanced', 'expert']).default("beginner"),
    programmingLanguages: z.array(z.string()),
    aiMlExperience: z.boolean().default(false),
    rosExperience: z.boolean().default(false),
    hardwareAccess: z.enum(['none', 'basic', 'moderate', 'advanced']).default("none"),
    gpuType: z.string(),
    hasRoboticsHardware: z.boolean().default(false),
    roboticsHardwareDetails: z.string(),
    learningGoals: z.array(z.string()),
});

export type UserProfileFormData = z.infer<typeof userProfileSchema>
