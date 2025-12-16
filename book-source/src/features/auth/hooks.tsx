import { useState } from 'react';
import { SignInFormData, SignUpFormData, UserProfileFormData } from './schema';
import { authClient } from '@site/src/lib/auth-client';


export const useAuth = () => {

  const [loading, setIsLoading] = useState(false);
  const [user, setUser] = useState(null);


  const signIn = async (data: SignInFormData) => {
    return { data: null, error: null }
  }

  const signOut = async () => {
    try {
      // Sign out using the auth client
      await authClient.signOut();

      // Clear local state
      setUser(null);

      // Clear localStorage
      localStorage.removeItem('better-auth-session');
      localStorage.removeItem('userBackground');
    } catch (error) {
      console.error('Logout error:', error);
    }
  }

  const signUp = async (data: SignUpFormData) => {
    setIsLoading(true);

    try {
      // Create user with Better Auth client
      const response = await authClient.signUp.email({
        email: data.email,
        password: data.password,
        name: data.name,
      });

      if (response?.error) {
        throw new Error(response.error.message || 'Signup failed');
      }

      // After successful signup, update user with background information
      // This would typically be done via an API call to update custom user fields
      // For now, we'll store this in localStorage as a placeholder
      const userData = {
        softwareExperience: data.softwareExperience,
        hardwareAccess: data.hardwareAccess,
        programmingLanguages: data.programmingLanguages,
        primaryFocus: data.primaryFocus,
        projectGoals: data.projectGoals,
        skillLevel: data.skillLevel,
      };

      localStorage.setItem('userBackground', JSON.stringify(userData));

      return { data: userData, error: null }

    } catch (err: any) {
      console.error('Signup error:', err);
      const errorMessage = err.message || 'An error occurred during signup';
      return { data: null, error: errorMessage }
    } finally {
      setIsLoading(false);
    }
  };

  return {
    signUp,
    signIn,
    signOut,
    loading,
    user,
  };
};

export const useProfile = () => {

  return {
    profile: null,
    loading: false,
    updateProfile: async (data: UserProfileFormData) => { return { error: null } },
  }
}
