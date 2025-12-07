import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { SignInFormData, signInSchema } from '@site/src/features/auth/schema';
import { useAuth } from '@site/src/features/auth/hooks';
import { Button } from '../ui/button';
import { Loader2Icon } from 'lucide-react';
import { Input } from '../ui/input';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '../ui/form';


interface SignInFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export const SignInForm: React.FC<SignInFormProps> = ({ onSuccess, onError }) => {

  const { signIn, loading } = useAuth();
  const [error, setError] = useState<string | null>(null);

  const form = useForm<SignInFormData>({
    resolver: zodResolver(signInSchema),
  });

  const onSubmit = async (values: SignInFormData) => {
    setError(null);

    try {
      // Create user with Better Auth client
      const { data, error } = await signIn(values)
      // Call the success callback
      onSuccess?.();
    } catch (err: any) {
      console.error('SignIn error:', err);
      const errorMessage = err.message || 'An error occurred during sign in';
      setError(errorMessage);
      onError?.(errorMessage);
    } finally {
    }
  };


  return (
    <Form {...form}>
      <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">

        <FormField
          name="email"
          control={form.control}
          render={({ field }) => (
            <FormItem className="space-y-2">
              <FormLabel>Email</FormLabel>
              <FormControl>
                <Input
                  {...field}
                  type="email"
                  placeholder="you@example.com"
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />

        <FormField
          control={form.control}
          name="password"
          render={({ field }) => (
            <FormItem className="space-y-2">
              <FormLabel>Password</FormLabel>
              <FormControl>
                <Input
                  {...field}
                  type="password"
                  placeholder="••••••••"
                />
              </FormControl>
              <FormMessage />
            </FormItem>
          )}
        />
        <Button type="submit" className="w-full" disabled={loading}>
          {loading ? <Loader2Icon className="mr-2 h-4 w-4 animate-spin" /> : null}
          Sign In
        </Button>
      </form>
    </Form>
  );
};
