import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { authClient } from '../../lib/auth-client';
import AuthModal from '../AuthModal';
import styles from './styles.module.css';

export default function NavbarAuth(): React.JSX.Element {
  const session = authClient.useSession();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const history = useHistory();

  const handleSignOut = async () => {
    await authClient.signOut();
    history.push('/'); // Redirect to home after logout
    window.location.reload(); // Reload to clear any client-side state/cache if needed
  };

  if (session.data) {
    return (
      <div className="navbar__item">
        <button
          onClick={handleSignOut}
          className="button button--secondary button--sm"
          style={{ fontWeight: 600 }}
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <>
      <div className="navbar__item">
        <button
          onClick={() => setShowAuthModal(true)}
          className="button button--primary button--sm"
          style={{ fontWeight: 600 }}
        >
          Sign In
        </button>
      </div>
      
      <AuthModal 
        isOpen={showAuthModal} 
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => {
            setShowAuthModal(false);
            window.location.reload(); // Reload to update UI state
        }}
      />
    </>
  );
}
