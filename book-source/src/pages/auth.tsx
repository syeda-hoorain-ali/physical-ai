import { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader } from '@site/src/components/ui/card';
import { Tabs, TabsList, TabsTrigger } from '@site/src/components/ui/tabs';
import { BookOpen, Loader2 } from 'lucide-react';
import { useAuth } from '../features/auth/hooks';
import { SignUpForm } from '../components/auth/sign-up-form';
import { SignInForm } from '../components/auth/sign-in-form';
import { useHistory } from '@docusaurus/router';
import { Navbar } from '../components/navbar';

const Auth = () => {

  const history = useHistory();
  const { user, loading: authLoading } = useAuth();

  const [activeTab, setActiveTab] = useState<'signin' | 'signup'>('signin');

  useEffect(() => {
    if (user && !authLoading) {
      history.push('/onboarding');
    }
  }, [user, authLoading]);


  if (authLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-background">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
      </div>
    );
  }

  return (<>
    <Navbar />

    <div className="flex min-h-screen items-center justify-center bg-background p-4 mt-16">
      <div className="w-full max-w-md space-y-8">
        <div className="text-center">
          <div className="mb-4 flex justify-center">
            <div className="rounded-xl bg-linear-to-br from-primary to-secondary p-3">
              <BookOpen className="h-8 w-8 text-primary-foreground" />
            </div>
          </div>
          <h1 className="text-2xl font-bold text-foreground">Physical AI Textbook</h1>
          <p className="mt-2 text-muted-foreground">
            Sign in to access personalized learning content
          </p>
        </div>

        <Card className="border-border/50 bg-card/80 backdrop-blur">
          <CardHeader className="pb-4">
            <Tabs value={activeTab} onValueChange={(v) => setActiveTab(v as 'signin' | 'signup')}>
              <TabsList className="grid w-full grid-cols-2">
                <TabsTrigger value="signin">Sign In</TabsTrigger>
                <TabsTrigger value="signup">Sign Up</TabsTrigger>
              </TabsList>
            </Tabs>
          </CardHeader>

          <CardContent>
            {activeTab === 'signin' ? (
              <SignInForm />
            ) : (
              <SignUpForm />
            )}
          </CardContent>
        </Card>

        <p className="text-center text-sm text-muted-foreground">
          <a href="/" className="text-primary hover:underline">
            ← Back to Home
          </a>
        </p>
      </div>
    </div>
  </>);
};

export default Auth;
