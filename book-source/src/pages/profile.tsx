import { useEffect } from 'react';
import { useAuth, useProfile } from '@site/src/features/auth/hooks';
import { UserProfile } from '@site/src/components/auth/user-profile';
import { Loader2Icon } from 'lucide-react';
import { useHistory } from '@docusaurus/router';


const Profile = () => {

  const history = useHistory();
  const { user, loading: authLoading } = useAuth();
  const { loading: profileLoading } = useProfile();


  useEffect(() => {
    if (!authLoading && !user) {
      history.push('/auth');
    }
  }, [user, authLoading]);


  if (authLoading || profileLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-background">
        <Loader2Icon className="h-8 w-8 animate-spin text-primary" />
      </div>
    );
  }

  return (
    <UserProfile />
  );
};

export default Profile;
